// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "./mcl_ros.hpp"
#include "./message_forward.hpp"
#include "dynamic_reconfigure/server.h"
#include "message_filters/subscriber.h"
#include "ruvu_mcl/Params.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

// forward declare
namespace geometry_msgs::msg
{
ROS_DECLARE_MESSAGE(PoseWithCovarianceStamped)
}
namespace nav_msgs::msg
{
ROS_DECLARE_MESSAGE(OccupancyGrid)
}
namespace sensor_msgs::msg
{
ROS_DECLARE_MESSAGE(LaserScan)
}

namespace ruvu_mcl
{
/**
 * @brief Main ros node of the ruvu_mcl package
 */
class Node
{
public:
  Node(rclcpp::Node nh, rclcpp::Node private_nh);
  ~Node();  // to handle forward declares

private:
  void scan_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);
  void landmark_cb(const ruvu_mcl_msgs::msg::LandmarkListConstPtr & landmarks);
  void map_cb(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map);
  void landmark_list_cb(const ruvu_mcl_msgs::msg::LandmarkListConstPtr & landmark_list);
  void initial_pose_cb(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & initial_pose);

  // data input
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> laser_scan_filter_;
  message_filters::Subscriber<ruvu_mcl_msgs::msg::LandmarkList> landmark_sub_;
  tf2_ros::MessageFilter<ruvu_mcl_msgs::msg::LandmarkList> landmark_filter_;
  ros::Subscriber map_sub_;
  ros::Subscriber landmark_list_sub_;
  ros::Subscriber initial_pose_sub_;
  dynamic_reconfigure::Server<ruvu_mcl::Params> reconfigure_server_;

  // internals
  MclRos filter_;
};
}  // namespace ruvu_mcl
