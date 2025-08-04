// Copyright 2021 RUVU Robotics B.V.

#include "./mcl_ros.hpp"

#include <algorithm>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include "./sensor_models/landmark.hpp"
#include "./sensor_models/laser.hpp"
#include "ruvu_mcl_msgs/msg/landmark_list.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

constexpr auto name = "mcl_ros";

namespace ruvu_mcl
{
geometry_msgs::msg::PoseWithCovariance to_msg(PoseWithCovariance pose_with_covariance)
{
  geometry_msgs::msg::PoseWithCovariance ps;
  tf2::toMsg(pose_with_covariance.pose.getOrigin(), ps.pose.position);
  tf2::convert(pose_with_covariance.pose.getRotation(), ps.pose.orientation);
  assert(pose_with_covariance.covariance.size() == ps.covariance.size());
  std::copy(
    pose_with_covariance.covariance.begin(), pose_with_covariance.covariance.end(),
    ps.covariance.begin());
  return ps;
}

MclRos::MclRos(rclcpp::Node::SharedPtr nh, const std::shared_ptr<const tf2::BufferCore> & buffer)
: mcl_(nh),
  buffer_(buffer),
  transform_br_(nh),
  last_tf_broadcast_(),
  cloud_pub_(nh),
  count_pub_(nh->create_publisher<std_msgs::msg::UInt32>("~/count", 1)),
  pose_pub_(nh->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 1))
{
}

MclRos::MclRos(
  rclcpp::Node::SharedPtr nh, const std::shared_ptr<const tf2::BufferCore> & buffer,
  uint_fast32_t seed)
: mcl_(nh, seed),
  buffer_(buffer),
  transform_br_(nh),
  last_tf_broadcast_(),
  cloud_pub_(nh),
  count_pub_(nh->create_publisher<std_msgs::msg::UInt32>("~/count", 1)),
  pose_pub_(nh->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 1))
{
}

MclRos::~MclRos() = default;

void MclRos::configure(const ruvu_mcl::Params & config) { mcl_.configure(Config{config}); }

const geometry_msgs::msg::PoseWithCovariance MclRos::get_pose_with_covariance() const
{
  auto ps = mcl_.get_pose_with_covariance();
  return to_msg(ps);
}

bool MclRos::scan_cb(const sensor_msgs::msg::LaserScan & scan)
{
  tf2::Transform tf;
  {
    auto nanoseconds = std::chrono::nanoseconds(rclcpp::Time{scan.header.stamp}.nanoseconds());
    auto tfs = buffer_->lookupTransform(
      mcl_.config().base_frame_id, scan.header.frame_id, tf2::TimePoint{nanoseconds});
    tf2::fromMsg(tfs.transform, tf);
  }
  LaserData data{scan, tf};

  tf2::Transform odom_pose;
  try {
    odom_pose = get_odom_pose(data.header.stamp);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(
      rclcpp::get_logger(name), "failed to compute odom pose, skipping measurement (%s)", e.what());
    return false;
  }

  auto updated = mcl_.scan_cb(data, odom_pose);

  if (updated) {
    auto ps = mcl_.get_pose_with_covariance();
    broadcast_tf(ps.pose, odom_pose, data.header.stamp);
    publish_data(data.header.stamp, ps);
  } else if (!last_tf_broadcast_.header.frame_id.empty()) {  // Verify last_tf_broadcast_ is set
    broadcast_last_tf(data.header.stamp);
  }
  return updated;
}

bool MclRos::landmark_cb(const ruvu_mcl_msgs::msg::LandmarkList & landmarks)
{
  tf2::Transform tf;
  {
    auto nanoseconds = std::chrono::nanoseconds(rclcpp::Time{landmarks.header.stamp}.nanoseconds());
    auto tfs = buffer_->lookupTransform(
      mcl_.config().base_frame_id, landmarks.header.frame_id, tf2::TimePoint{nanoseconds});
    tf2::fromMsg(tfs.transform, tf);
  }
  LandmarkList data{landmarks, tf};

  tf2::Transform odom_pose;
  try {
    odom_pose = get_odom_pose(data.header.stamp);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(
      rclcpp::get_logger(name), "failed to compute odom pose, skipping measurement (%s)", e.what());
    return false;
  }

  auto updated = mcl_.landmark_cb(data, odom_pose);

  if (updated) {
    auto ps = mcl_.get_pose_with_covariance();
    broadcast_tf(ps.pose, odom_pose, data.header.stamp);
    publish_data(data.header.stamp, ps);
  } else if (!last_tf_broadcast_.header.frame_id.empty()) {  // Verify last_tf_broadcast_ is set
    broadcast_last_tf(data.header.stamp);
  }
  return updated;
}

void MclRos::map_cb(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid> & map)
{
  mcl_.map_cb(map);
}

void MclRos::landmark_list_cb(const ruvu_mcl_msgs::msg::LandmarkList & landmarks)
{
  LandmarkList data{landmarks};
  mcl_.landmark_list_cb(data);
}

void MclRos::initial_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose)
{
  if (initial_pose.header.frame_id != mcl_.config().global_frame_id) {
    RCLCPP_WARN(rclcpp::get_logger(name), "initial pose is only accepted in the global frame");
  }

  // convert PoseWithCovariance to MCL datatype
  tf2::Transform p;
  tf2::convert(initial_pose.pose.pose.position, p.getOrigin());
  tf2::Quaternion q;
  tf2::convert(initial_pose.pose.pose.orientation, q);
  p.setRotation(q);
  std::array<double, 36> covariance;
  assert(initial_pose.pose.covariance.size() == covariance.size());
  std::copy(
    initial_pose.pose.covariance.begin(), initial_pose.pose.covariance.end(), covariance.begin());
  PoseWithCovariance pose_with_covariance{p, covariance};

  mcl_.initial_pose_cb(initial_pose.header.stamp, pose_with_covariance);
  mcl_.request_nomotion_update();
  publish_data(initial_pose.header.stamp, pose_with_covariance);
}

tf2::Transform MclRos::get_odom_pose(const rclcpp::Time & time) const
{
  // don't use .transform() because this could run offline without a listener thread
  auto nanoseconds = std::chrono::nanoseconds(time.nanoseconds());
  auto tf = buffer_->lookupTransform(
    mcl_.config().odom_frame_id, mcl_.config().base_frame_id, tf2::TimePoint{nanoseconds});
  tf2::Transform odom_pose_tf;
  tf2::fromMsg(tf.transform, odom_pose_tf);
  return odom_pose_tf;
}

void MclRos::broadcast_tf(
  const tf2::Transform & pose, const tf2::Transform & odom_pose, const rclcpp::Time & stamp)
{
  if (stamp <= last_tf_broadcast_.header.stamp) {
    return;
  }

  // Broadcast transform
  geometry_msgs::msg::TransformStamped transform_msg;

  transform_msg.header.stamp =
    stamp + rclcpp::Duration::from_seconds(mcl_.config().transform_tolerance);
  transform_msg.header.frame_id = mcl_.config().global_frame_id;
  transform_msg.child_frame_id = mcl_.config().odom_frame_id;
  transform_msg.transform = tf2::toMsg(pose * odom_pose.inverse());

  transform_br_.sendTransform(transform_msg);
  last_tf_broadcast_ = transform_msg;
}

void MclRos::broadcast_last_tf(const rclcpp::Time & stamp)
{
  auto msg = last_tf_broadcast_;
  msg.header.stamp = stamp + rclcpp::Duration::from_seconds(mcl_.config().transform_tolerance);

  transform_br_.sendTransform(msg);
  last_tf_broadcast_.header.stamp = stamp;
}

void MclRos::publish_data(
  const rclcpp::Time & stamp, const PoseWithCovariance & pose_with_covariance)
{
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = mcl_.config().global_frame_id;
  cloud_pub_.publish(header, mcl_.particles());

  std_msgs::msg::UInt32 count;
  count.data = mcl_.particles().size();
  count_pub_->publish(count);

  geometry_msgs::msg::PoseWithCovarianceStamped ps;
  ps.header = header;
  ps.pose = to_msg(pose_with_covariance);
  pose_pub_->publish(ps);
}
}  // namespace ruvu_mcl
