// Copyright (C) 2024 Nobleo Autonomous Solutions B.V.

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "./mcl_ros.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace ruvu_mcl
{
class MclComponent
{
public:
  explicit MclComponent(const rclcpp::NodeOptions & options)
  : node_(std::make_shared<rclcpp::Node>("mcl", options)),
    buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
    tf_listener_(*buffer_, node_),
    laser_scan_sub_(node_, "scan", rmw_qos_profile_sensor_data),
    laser_scan_filter_(laser_scan_sub_, *buffer_, "", 100, node_),
    mcl_ros_(node_, buffer_)
  {
    laser_scan_filter_.registerCallback(&MclComponent::scan_cb, this);
    // TODO(Ramon): Also add the landmark callbacks
  }

  void scan_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
  {
    mcl_ros_.scan_cb(*scan);
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
  {
    return node_->get_node_base_interface();
  }

private:
  // Input
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> laser_scan_filter_;

  // State
  MclRos mcl_ros_;

  // Output
};
}  // namespace ruvu_mcl

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ruvu_mcl::MclComponent)
