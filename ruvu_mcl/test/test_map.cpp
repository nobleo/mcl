// Copyright 2021 RUVU Robotics B.V.

#include <gtest/gtest.h>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "../src/map.hpp"
// #include "ros/console.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using ruvu_mcl::Map;

TEST(TestSuite, test_world2map_offset)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = 0.1;  // m/pixel
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::toMsg(tf2::Transform{q, tf2::Vector3{10, 10, 0}}, msg.info.origin);
  Map map{msg};
  {
    auto [i, j] = map.world2map({10, 10, 0});
    ASSERT_EQ(i, 0);
    ASSERT_EQ(j, 0);
  }
  {
    auto [i, j] = map.world2map({9, 12, 0});
    ASSERT_EQ(i, 20);
    ASSERT_EQ(j, 10);
  }
}

TEST(TestSuite, test_world2map_identity)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = 0.1;  // m/pixel
  tf2::toMsg(tf2::Transform::getIdentity(), msg.info.origin);

  Map map{msg};
  {
    auto [i, j] = map.world2map({0, 0, 0});
    ASSERT_EQ(i, 0);
    ASSERT_EQ(j, 0);
  }
  {
    auto [i, j] = map.world2map({1, 2, 0});
    ASSERT_EQ(i, 10);
    ASSERT_EQ(j, 20);
  }
}

TEST(TestSuite, test_world2map_rounding)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = 1;  // m/pixel
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::toMsg(tf2::Transform::getIdentity(), msg.info.origin);
  Map map{msg};
  {
    auto [i, j] = map.world2map({0.1, 0, 0});
    ASSERT_EQ(i, 0);
  }
  {
    auto [i, j] = map.world2map({0.9, 0, 0});
    ASSERT_EQ(i, 1);
  }
  {
    auto [i, j] = map.world2map({1.1, 0, 0});
    ASSERT_EQ(i, 1);
  }
  {
    auto [i, j] = map.world2map({-0.1, 0, 0});
    ASSERT_EQ(i, 0);
  }
  {
    auto [i, j] = map.world2map({-0.9, 0, 0});
    ASSERT_EQ(i, -1);
  }
  {
    auto [i, j] = map.world2map({-1.1, 0, 0});
    ASSERT_EQ(i, -1);
  }
}
