// Copyright 2021 RUVU Robotics B.V.

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <limits>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>

#include "./mcl_ros.hpp"
#include "./offline/bag_buffer.hpp"
#include "./offline/bag_player.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "ruvu_mcl/parameters.hpp"
#include "ruvu_mcl_msgs/msg/landmark_list.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using ruvu_mcl::BagPlayer;
using ruvu_mcl::create_bag_buffer;
using ruvu_mcl::MclRos;
using ruvu_mcl::ParamListener;
using ruvu_mcl::Params;

/**
 * @brief Executable for offline playback of the particle filter on ros bagfiles
 */
int main(int argc, char ** argv)
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  auto nh = rclcpp::Node::make_shared("mcl");
  auto logger = nh->get_logger();
  RCLCPP_INFO(logger, "%s started", nh->get_name());

  if (args.size() != 2) {
    puts("usage: offline BAGFILE");
    exit(EXIT_FAILURE);
  }

  BagPlayer player{args[1], nh->get_clock()};
  auto rate = nh->declare_parameter("rate", std::numeric_limits<double>::infinity());
  player.set_playback_speed(rate);

  auto buffer = create_bag_buffer(*player.bag);
  MclRos filter{nh, buffer};

  ParamListener listener{nh};
  listener.setUserCallback([&](const Params & params) {
    RCLCPP_INFO(logger, "reconfigure call");
    filter.configure(params);
  });
  filter.configure(listener.get_params());

  auto scan_pub = nh->create_publisher<sensor_msgs::msg::LaserScan>("scan", 100);
  auto landmark_pub = nh->create_publisher<ruvu_mcl_msgs::msg::LandmarkList>("landmarks", 100);
  auto map_pub =
    nh->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS{1}.transient_local());
  auto landmark_list_pub = nh->create_publisher<ruvu_mcl_msgs::msg::LandmarkList>(
    "landmark_list", rclcpp::QoS{1}.transient_local());
  auto tf_pub = nh->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
  auto tf_static_pub = nh->create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS{100}.transient_local());

  rclcpp::WallRate r{10};
  r.sleep();  // wait for topics to connect

  player.register_callback<sensor_msgs::msg::LaserScan>(
    "/scan", [&filter, &scan_pub](const sensor_msgs::msg::LaserScan & scan) {
      scan_pub->publish(scan);
      filter.scan_cb(scan);
    });

  player.register_callback<ruvu_mcl_msgs::msg::LandmarkList>(
    "/landmarks", [&filter, &landmark_pub](const ruvu_mcl_msgs::msg::LandmarkList & landmarks) {
      landmark_pub->publish(landmarks);
      filter.landmark_cb(landmarks);
    });

  player.register_callback<nav_msgs::msg::OccupancyGrid>(
    "/map", [&filter, &map_pub](const nav_msgs::msg::OccupancyGrid & map) {
      map_pub->publish(map);
      filter.map_cb(std::make_shared<nav_msgs::msg::OccupancyGrid>(map));
    });

  player.register_callback<ruvu_mcl_msgs::msg::LandmarkList>(
    "/landmark_list",
    [&filter, &landmark_list_pub](const ruvu_mcl_msgs::msg::LandmarkList & landmark_list) {
      landmark_list_pub->publish(landmark_list);
      filter.landmark_list_cb(landmark_list);
    });

  player.register_callback<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", [&filter](const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose) {
      filter.initial_pose_cb(initial_pose);
    });

  player.register_callback<tf2_msgs::msg::TFMessage>(
    "/tf", [&tf_pub](const tf2_msgs::msg::TFMessage & tf) { tf_pub->publish(tf); });

  player.register_callback<tf2_msgs::msg::TFMessage>(
    "/tf_static",
    [&tf_static_pub](const tf2_msgs::msg::TFMessage & tf) { tf_static_pub->publish(tf); });

  player.start_play();

  RCLCPP_INFO(logger, "%s finished", nh->get_name());
  return EXIT_SUCCESS;
}
