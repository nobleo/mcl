// Copyright 2021 RUVU Robotics B.V.

#include "./cloud_publisher.hpp"

#include <algorithm>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "./particle_filter.hpp"
#include "rclcpp/node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ruvu_mcl
{
CloudPublisher::CloudPublisher(rclcpp::Node::SharedPtr nh)
: cloud_pub_(nh->create_publisher<visualization_msgs::msg::Marker>("~/cloud", 1))
{
}

void CloudPublisher::publish(const std_msgs::msg::Header & header, const std::vector<Particle> & pf)
{
  constexpr double length = 0.1;
  constexpr double width = 0.02;

  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns = "particles";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_LIST;
  m.action = visualization_msgs::msg::Marker::MODIFY;
  m.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
  m.scale.x = width;
  m.points.reserve(pf.size() * 2);
  m.colors.reserve(pf.size() * 2);

  double max_weight = 0;
  for (const auto & p : pf) max_weight = std::max(max_weight, p.weight);

  tf2::Vector3 p1{length / 2, 0, 0};
  for (const auto & particle : pf) {
    std_msgs::msg::ColorRGBA c;
    c.a = particle.weight / max_weight;
    c.b = 1;
    m.colors.push_back(c);
    m.colors.push_back(c);

    geometry_msgs::msg::Point front;
    tf2::toMsg(particle.pose * p1, front);
    m.points.push_back(front);

    geometry_msgs::msg::Point back;
    tf2::toMsg(particle.pose * -p1, back);
    m.points.push_back(back);
  }

  cloud_pub_->publish(m);
}
}  // namespace ruvu_mcl
