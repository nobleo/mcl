// Copyright 2021 RUVU Robotics B.V.

#include "./node.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>

#include "ruvu_mcl_msgs/LandmarkList.h"

constexpr auto name = "node";

namespace ruvu_mcl
{
Node::Node(rclcpp::Node nh, rclcpp::Node private_nh)
: buffer_(std::make_shared<tf2_ros::Buffer>()),
  tf_listener_(*buffer_),
  laser_scan_sub_(nh, "scan", 100),
  laser_scan_filter_(laser_scan_sub_, *buffer_, "", 100, nh),
  landmark_sub_(nh, "landmarks", 100),
  landmark_filter_(landmark_sub_, *buffer_, "", 100, nh),
  map_sub_(nh.subscribe<nav_msgs::msg::OccupancyGrid>("map", 1, &Node::map_cb, this)),
  landmark_list_sub_(nh.subscribe<ruvu_mcl_msgs::msg::LandmarkList>(
    "landmark_list", 1, &Node::landmark_list_cb, this)),
  initial_pose_sub_(nh.subscribe<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, &Node::initial_pose_cb, this)),
  reconfigure_server_(private_nh),
  filter_(nh, private_nh, buffer_)
{
  laser_scan_filter_.registerCallback(&Node::scan_cb, this);
  landmark_filter_.registerCallback(&Node::landmark_cb, this);
  reconfigure_server_.setCallback([this](const ruvu_mcl::Params & config, uint32_t level) {
    RCLCPP_INFO(rclcpp::get_logger(name), "reconfigure call");
    laser_scan_filter_.setTargetFrame(config.odom_frame_id);
    landmark_filter_.setTargetFrame(config.odom_frame_id);
    filter_.configure(config);
  });
}

Node::~Node() = default;

void Node::scan_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  filter_.scan_cb(scan);
}

void Node::landmark_cb(const ruvu_mcl_msgs::msg::LandmarkListConstPtr & landmarks)
{
  filter_.landmark_cb(landmarks);
}

void Node::map_cb(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map) { filter_.map_cb(map); }

void Node::landmark_list_cb(const ruvu_mcl_msgs::msg::LandmarkListConstPtr & landmark_list)
{
  filter_.landmark_list_cb(landmark_list);
}

void Node::initial_pose_cb(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & initial_pose)
{
  filter_.initial_pose_cb(initial_pose);
}
}  // namespace ruvu_mcl
