// Copyright 2021 RUVU Robotics B.V.

#include "./mcl.hpp"

#include <memory>

#include "./adaptive/fixed.hpp"
#include "./adaptive/kld_sampling.hpp"
#include "./adaptive/split_and_merge.hpp"
#include "./map.hpp"
#include "./motion_models/differential_motion_model.hpp"
#include "./resamplers/low_variance.hpp"
#include "./rng.hpp"
#include "./sensor_models/beam_model.hpp"
#include "./sensor_models/gaussian_landmark_model.hpp"
#include "./sensor_models/landmark_likelihood_field_model.hpp"
#include "./sensor_models/likelihood_field_model.hpp"
#include "rclcpp/node.hpp"
#include "tf2/utils.h"

constexpr auto name = "mcl";

namespace ruvu_mcl
{
std::unique_ptr<Laser> create_laser_model(
  rclcpp::Node::SharedPtr node, const Config & config,
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map)
{
  RCLCPP_INFO(rclcpp::get_logger(name), "adding a laser sensor model");
  if (const auto * c = std::get_if<BeamModelConfig>(&config.laser))
    return std::make_unique<BeamModel>(node, *c, std::make_shared<OccupancyMap>(*map));
  else if (const auto * c = std::get_if<LikelihoodFieldModelConfig>(&config.laser))
    return std::make_unique<LikelihoodFieldModel>(
      node, *c, std::make_shared<DistanceMap>(node, *map));
  else
    throw std::logic_error("no laser model configured");
}

Mcl::Mcl(const rclcpp::Node::SharedPtr & node) : Mcl(node, std::make_shared<Rng>()) {}

Mcl::Mcl(const rclcpp::Node::SharedPtr & node, uint_fast32_t seed)
: Mcl(node, std::make_shared<Rng>(seed))
{
}

Mcl::Mcl(const rclcpp::Node::SharedPtr & node, const std::shared_ptr<Rng> & rng)
: node_(node),
  config_(),
  rng_(rng),
  last_odom_pose_(),
  last_filter_update_(),
  filter_(),
  model_(nullptr),
  map_(nullptr),
  landmarks_(nullptr),
  laser_(nullptr),
  landmark_model_(nullptr),
  should_process_(),
  resampler_(std::make_unique<LowVariance>(rng_)),
  resample_count_(0),
  adaptive_(nullptr)
{
}

void Mcl::configure(const Config & config)
{
  RCLCPP_INFO(rclcpp::get_logger(name), "configure call");
  config_ = config;

  // they will configure themself on next scan_cb
  laser_ = nullptr;
  landmark_model_ = nullptr;

  if (const auto * c = std::get_if<DifferentialMotionModelConfig>(&config.model))
    model_ = std::make_unique<DifferentialMotionModel>(*c, rng_);
  else
    throw std::logic_error("no motion model configured");

  if (std::holds_alternative<FixedConfig>(config.adaptive))
    adaptive_ = std::make_unique<Fixed>(config);
  else if (const auto * c = std::get_if<KLDSamplingConfig>(&config.adaptive))
    adaptive_ = std::make_unique<KLDSampling>(*c);
  else if (std::holds_alternative<SplitAndMergeConfig>(config.adaptive))
    adaptive_ = std::make_unique<SplitAndMerge>(config);
  else
    throw std::logic_error("no adaptive algorithm configured");

  if (filter_.particles.empty()) {
    PoseWithCovariance p{config.initial_pose, config.initial_cov};
    initial_pose_cb(node_->get_clock()->now(), p);
  }
}

Mcl::~Mcl() = default;

bool Mcl::scan_cb(const LaserData & scan, const tf2::Transform & odom_pose)
{
  if (!odometry_update(scan.header, MeasurementType::LASER, odom_pose)) return false;

  assert(adaptive_);
  adaptive_->after_odometry_update(&filter_);

  if (!map_) {
    RCLCPP_WARN(rclcpp::get_logger(name), "no map yet received, skipping sensor model");
    return false;
  }
  if (!laser_) {
    laser_ = create_laser_model(node_, config_, map_);
  }

  laser_->sensor_update(&filter_, scan);

  adaptive_->after_sensor_update(&filter_);

  auto needed_particles = adaptive_->calc_needed_particles(filter_);

  // Resample
  if (config_.selective_resampling) {
    if (filter_.calc_effective_sample_size() < filter_.particles.size() / 2.)
      resampler_->resample(&filter_, needed_particles);
  } else {
    if (!(++resample_count_ % config_.resample_interval))
      resampler_->resample(&filter_, needed_particles);
  }

  last_filter_update_ = scan.header.stamp;
  return true;
}

bool Mcl::landmark_cb(const LandmarkList & landmarks, const tf2::Transform & odom_pose)
{
  if (!odometry_update(landmarks.header, MeasurementType::LANDMARK, odom_pose)) return false;

  assert(adaptive_);
  adaptive_->after_odometry_update(&filter_);

  if (!landmarks_) {
    RCLCPP_WARN(rclcpp::get_logger(name), "no landmark list yet received, skipping sensor model");
    return false;
  }
  if (!landmark_model_) {
    RCLCPP_INFO(rclcpp::get_logger(name), "adding a landmark sensor model");
    if (const auto * c = std::get_if<GaussianLandmarkModelConfig>(&config_.landmark))
      landmark_model_ = std::make_unique<GaussianLandmarkModel>(node_, *c, *landmarks_);
    else if (const auto * c = std::get_if<LandmarkLikelihoodFieldModelConfig>(&config_.landmark))
      landmark_model_ = std::make_unique<LandmarkLikelihoodFieldModel>(node_, *c, *landmarks_);
    else
      throw std::logic_error("no landmark model configured");
  }

  landmark_model_->sensor_update(&filter_, landmarks);

  adaptive_->after_sensor_update(&filter_);

  auto needed_particles = adaptive_->calc_needed_particles(filter_);

  // Resample
  if (config_.selective_resampling) {
    if (filter_.calc_effective_sample_size() < filter_.particles.size() / 2.)
      resampler_->resample(&filter_, needed_particles);
  } else {
    if (!(++resample_count_ % config_.resample_interval))
      resampler_->resample(&filter_, needed_particles);
  }

  last_filter_update_ = landmarks.header.stamp;
  return true;
}

void Mcl::map_cb(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid> & map)
{
  RCLCPP_INFO(rclcpp::get_logger(name), "map received");
  map_ = map;
  laser_ = nullptr;
}

void Mcl::landmark_list_cb(const LandmarkList & landmarks)
{
  RCLCPP_INFO(rclcpp::get_logger(name), "landmark list received");
  landmarks_ = std::make_shared<LandmarkList>(landmarks);
  landmark_model_ = nullptr;
}

void Mcl::initial_pose_cb(const rclcpp::Time & stamp, const PoseWithCovariance & initial_pose)
{
  const auto & p = initial_pose.pose;
  const auto & covariance = initial_pose.covariance;
  RCLCPP_INFO(
    rclcpp::get_logger(name), "initial pose received %.3f %.3f %.3f, spawning %lu new particles",
    p.getOrigin().x(), p.getOrigin().y(), tf2::getYaw(p.getRotation()), config_.max_particles);

  auto dx = rng_->normal_distribution(p.getOrigin().x(), covariance[0 * 6 + 0]);
  auto dy = rng_->normal_distribution(p.getOrigin().y(), covariance[1 * 6 + 1]);
  auto dt = rng_->normal_distribution(tf2::getYaw(p.getRotation()), covariance[5 * 6 + 5]);

  filter_.particles.clear();
  for (size_t i = 0; i < config_.max_particles; ++i) {
    tf2::Vector3 p{dx(), dy(), 0};
    tf2::Quaternion q;
    q.setRPY(0, 0, dt());
    filter_.particles.emplace_back(tf2::Transform{q, p}, 1. / config_.max_particles);
  }

  last_filter_update_ = stamp;
}

void Mcl::request_nomotion_update()
{
  for (auto & key : should_process_) {
    key.second = true;
  }
}

bool Mcl::odometry_update(
  const std_msgs::msg::Header & header, const MeasurementType & measurement_type,
  tf2::Transform odom_pose)
{
  assert(model_);

  if (!last_odom_pose_) {
    RCLCPP_INFO(rclcpp::get_logger(name), "first odometry update, recording the odom pose");
    last_odom_pose_ = odom_pose;
    last_filter_update_ = header.stamp;
    // call should_process to record the frame_id
    should_process(tf2::Transform::getIdentity(), {measurement_type, header.frame_id});
    return true;
  }

  if (rclcpp::Time{header.stamp} <= last_filter_update_) {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(), "skipping out-of-order measurement, "
                             << rclcpp::Time{header.stamp}.seconds()
                             << " <= " << last_filter_update_.seconds());
    return false;
  }

  auto diff = last_odom_pose_->inverseTimes(odom_pose);

  if (!should_process(diff, {measurement_type, header.frame_id})) {
    return false;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger(name), "movement: x=%f y=%f t=%f", diff.getOrigin().getX(),
    diff.getOrigin().getY(), tf2::getYaw(diff.getRotation()));

  model_->odometry_update(&filter_, diff);

  last_odom_pose_ = odom_pose;
  return true;
}

bool Mcl::should_process(const tf2::Transform & diff, const MeasurementKey & measurment_key)
{
  if (
    diff.getOrigin().length() >= config_.update_min_d ||
    fabs(tf2::getYaw(diff.getRotation())) >= config_.update_min_a) {
    RCLCPP_DEBUG(rclcpp::get_logger(name), "enough movement detected, processing all sensors");
    for (auto & s : should_process_) {
      s.second = true;
    }
  }

  if (should_process_.find(measurment_key) == should_process_.end()) {
    should_process_[measurment_key] = true;
  }

  if (should_process_[measurment_key]) {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      "processing " << std::get<0>(measurment_key) << ' ' << std::get<1>(measurment_key));
    should_process_[measurment_key] = false;
    return true;
  } else {
    return false;
  }
}

std::ostream & operator<<(std::ostream & out, const Mcl::MeasurementType & measurement_type)
{
  switch (measurement_type) {
    case Mcl::MeasurementType::LASER:
      return out << "laser";
    case Mcl::MeasurementType::LANDMARK:
      return out << "landmark";
    default:
      throw std::logic_error("unknown measurement type");
  }
}
}  // namespace ruvu_mcl
