// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "./cloud_publisher.hpp"
#include "./mcl.hpp"
#include "./message_forward.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace ros
{
class NodeHandle;
}
namespace tf2
{
class BufferCore;
}
namespace nav_msgs::msg
{
ROS_DECLARE_MESSAGE(OccupancyGrid)
}
namespace sensor_msgs::msg
{
ROS_DECLARE_MESSAGE(LaserScan)
}
namespace ruvu_mcl_msgs::msg
{
ROS_DECLARE_MESSAGE(LandmarkList)
}

namespace ruvu_mcl
{
class Params;

/**
 * @brief ROS wrapper for the Mcl class
 *
 * This class is a ROS wrapper for the Mcl class. It receives ROS messages, converts then and sends
 * then trough. It also publishes the output of Mcl to /tf and ~pose.
 */
class MclRos
{
public:
  MclRos(rclcpp::Node::SharedPtr nh, const std::shared_ptr<const tf2::BufferCore> & buffer);
  MclRos(
    rclcpp::Node::SharedPtr nh, const std::shared_ptr<const tf2::BufferCore> & buffer,
    uint_fast32_t seed);
  ~MclRos();  // to handle forward declares

  void configure(const ruvu_mcl::Params & config);
  const geometry_msgs::msg::PoseWithCovariance get_pose_with_covariance() const;

  bool scan_cb(const sensor_msgs::msg::LaserScan & scan);
  bool landmark_cb(const ruvu_mcl_msgs::msg::LandmarkList & landmarks);
  void map_cb(const std::shared_ptr<const nav_msgs::msg::OccupancyGrid> & map);
  void landmark_list_cb(const ruvu_mcl_msgs::msg::LandmarkList & landmarks);
  void initial_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose);

private:
  tf2::Transform get_odom_pose(const rclcpp::Time & time) const;
  void broadcast_tf(
    const tf2::Transform & pose, const tf2::Transform & odom_pose, const rclcpp::Time & stamp);
  void broadcast_last_tf(const rclcpp::Time & stamp);
  void publish_data(const rclcpp::Time & stamp, const PoseWithCovariance & pose_with_covariance);

  Mcl mcl_;
  std::shared_ptr<const tf2::BufferCore> buffer_;
  tf2_ros::TransformBroadcaster transform_br_;
  geometry_msgs::msg::TransformStamped last_tf_broadcast_;
  CloudPublisher cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};
}  // namespace ruvu_mcl
