// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "tf2/buffer_core.h"

namespace rosbag2_cpp
{
class Reader;
}

namespace ruvu_mcl
{
/**
 * @brief Load all tf messages from a rosbag into a tf2_ros::Buffer
 */
std::shared_ptr<const tf2::BufferCore> create_bag_buffer(rosbag2_cpp::Reader & bag);
}  // namespace ruvu_mcl
