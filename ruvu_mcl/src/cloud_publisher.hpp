// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <vector>

#include "./message_forward.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace ros
{
class Particle;
}
namespace std_msgs::msg
{
ROS_DECLARE_MESSAGE(Header)
}

namespace ruvu_mcl
{
class Particle;

/**
 * @brief Publishes a vector of particles as a visualization_msgs::msg::Marker
 */
class CloudPublisher
{
public:
  CloudPublisher(rclcpp::Node::SharedPtr nh);
  void publish(const std_msgs::msg::Header & header, const std::vector<Particle> & pf);

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cloud_pub_;
};
}  // namespace ruvu_mcl
