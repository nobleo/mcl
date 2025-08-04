// Copyright 2021 RUVU Robotics B.V.

#include <gtest/gtest.h>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "../src/map.hpp"
#include "rclcpp/logging.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using ruvu_mcl::OccupancyMap;

TEST(TestSuite, testCells)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = 3;
  msg.info.height = 2;
  msg.data = {0, 100, -1, 100, 100, 100};
  //          -1,  1,  0,   1,   1,   1
  /**
   * This is the storage order of an OccupancyGrid
   *   y (cols, height) ->
   * x 0 3
   *   1 4
   *   2 5
   */

  OccupancyMap map{msg};

  std::cout << "In memory storage:\n";
  for (int i = 0; i < map.cells.size(); i++)
    std::cout << static_cast<int>(map.cells.data()[i]) << "  ";
  std::cout << '\n';

  ASSERT_EQ(map.cells.rows(), msg.info.width);
  ASSERT_EQ(map.cells.cols(), msg.info.height);

  // Let's check the map data
  ASSERT_EQ(map.cells(0, 0), -1);
  ASSERT_EQ(map.cells(1, 0), 1);
  ASSERT_EQ(map.cells(2, 0), 0);
  ASSERT_EQ(map.cells(0, 1), 1);
  ASSERT_EQ(map.cells(1, 1), 1);
  ASSERT_EQ(map.cells(2, 1), 1);
}

TEST(TestSuite, test_is_valid)
{
  // set the origin with an offset and turned 90 degree
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = 3;
  msg.info.height = 2;
  msg.data.resize(msg.info.width * msg.info.height);

  OccupancyMap map{msg};
  ASSERT_TRUE(map.is_valid(0, 0));
  ASSERT_TRUE(map.is_valid(2, 1));  // the corner
  ASSERT_FALSE(map.is_valid(3, 1));
  ASSERT_FALSE(map.is_valid(2, 2));
  ASSERT_FALSE(map.is_valid(-1, 0));
  ASSERT_FALSE(map.is_valid(0, -1));
}
