// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <kdtree++/kdtree.hpp>

#include "../message_forward.hpp"
#include "./landmark.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "ruvu_mcl_msgs/msg/particle_statistics.hpp"
#include "visualization_msgs/msg/marker.hpp"

// forward declare
namespace ruvu_mcl_msgs::msg
{
ROS_DECLARE_MESSAGE(LandmarkList)
}

namespace ruvu_mcl
{
class GaussianLandmarkModelConfig;

struct KDTreeNode
{
  typedef double value_type;
  KDTreeNode(const Landmark & landmark) : landmark(landmark) {}
  value_type operator[](size_t n) const { return landmark.pose.getOrigin()[n]; }
  Landmark landmark;
};

using KDTreeType = KDTree::KDTree<2, KDTreeNode>;

/**
 * @brief Implements the landmark model known correspondence from Probablistic Robotics
 */
class GaussianLandmarkModel : public LandmarkModel
{
public:
  /*
   * @brief GaussianLandmarkModel constructor
   */
  GaussianLandmarkModel(
    rclcpp::Node::SharedPtr nh, const GaussianLandmarkModelConfig & config,
    const LandmarkList & map);

  void sensor_update(ParticleFilter * pf, const LandmarkList & data);

private:
  // parameters
  double z_rand_;
  double landmark_var_r_;
  double landmark_var_t_;
  double max_confidence_range_;
  std::string global_frame_id_;

  KDTreeType tree_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_pub_;
  rclcpp::Publisher<ruvu_mcl_msgs::msg::ParticleStatistics>::SharedPtr statistics_pub_;
};
}  // namespace ruvu_mcl
