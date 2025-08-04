// Copyright 2021 RUVU Robotics B.V.

#include "./node.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "ros/init.h"

using ruvu_mcl::Node;

constexpr auto name = "main";

/**
* @brief Main ros node of the ruvu_mcl package
*/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("mcl");
  rclcpp::Node private_nh{"~"};
  Node node{nh, private_nh};
  RCLCPP_INFO(rclcpp::get_logger(name), "%s started", private_nh.getNamespace().c_str());
  rclcpp::spin(node);
  return EXIT_SUCCESS;
}
