// Copyright 2021 RUVU Robotics B.V.

#include <Magick++.h>

#include <iostream>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "../src/map.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using ruvu_mcl::OccupancyMap;

Magick::Image cells_to_image(const OccupancyMap::CellsType & cells)
{
  Magick::Image img;
  img.size(Magick::Geometry{
    static_cast<unsigned int>(cells.rows()), static_cast<unsigned int>(cells.cols())});

  for (std::size_t x = 0; x < cells.rows(); ++x) {
    for (std::size_t y = 0; y < cells.cols(); ++y) {
      auto occ = cells(x, y);
      Magick::Color color;
      if (occ < 0) {  // free
        color = Magick::ColorGray(254 / 255.0);
      } else if (occ == 0) {  // unknown
        color = Magick::ColorGray(205 / 255.0);
      } else {  // occ
        color = Magick::ColorGray(0 / 255.0);
      }
      img.pixelColor(x, y, color);
    }
  }
  return img;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("render_scan");

  nav_msgs::msg::OccupancyGrid msg;
  rclcpp::wait_for_message(msg, nh, "map");
  OccupancyMap map{msg};

  auto range = map.calc_range(10, 10, 100, 100);
  RCLCPP_INFO_STREAM(nh->get_logger(), "range: " << range);

  Magick::InitializeMagick(nullptr);
  auto img = cells_to_image(map.cells);
  img.display();
  return EXIT_SUCCESS;
}
