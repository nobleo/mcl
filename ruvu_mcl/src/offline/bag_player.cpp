/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Open Source Robotics Foundation
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "./bag_player.hpp"

#include <rosbag2_transport/reader_writer_factory.hpp>

#include "rosbag2_cpp/reader.hpp"

namespace ruvu_mcl
{
rclcpp::Time chrono_to_rclcpp_time(const std::chrono::system_clock::time_point & tp)
{
  return rclcpp::Time(
    std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count());
}

BagPlayer::BagPlayer(const std::string & filename, rclcpp::Clock::SharedPtr clock) : clock_(clock)
{
  rosbag2_storage::StorageOptions reader_storage_options;
  reader_storage_options.uri = filename;
  bag = rosbag2_transport::ReaderWriterFactory::make_reader(reader_storage_options);
  bag->open(reader_storage_options);

  auto metadata = bag->get_metadata();
  bag_start_ = chrono_to_rclcpp_time(metadata.starting_time);
  bag_end_ = chrono_to_rclcpp_time(metadata.starting_time + metadata.duration);
  last_message_time_ = rclcpp::Time(0);
  playback_speed_ = 1.0;
}

BagPlayer::~BagPlayer() {}

void BagPlayer::set_playback_speed(double scale)
{
  if (scale > 0.0) playback_speed_ = scale;
}

rclcpp::Time BagPlayer::real_time(const rclcpp::Time & msg_time) const
{
  return play_start_ + (msg_time - bag_start_) * (1 / playback_speed_);
}

void BagPlayer::start_play()
{
  std::vector<std::string> topics;
  for (const auto & cb : cbs_) topics.push_back(cb.first);

  rosbag2_storage::StorageFilter filter;
  filter.topics = topics;
  bag->set_filter(filter);

  play_start_ = clock_->now();

  while (bag->has_next() and rclcpp::ok()) {
    auto msg = bag->read_next();

    if (cbs_.find(msg->topic_name) == cbs_.end()) continue;

    clock_->sleep_until(real_time(rclcpp::Time(msg->send_timestamp)));

    // TODO(Ramon): spin

    last_message_time_ = rclcpp::Time(msg->send_timestamp);
    auto cb = cbs_[msg->topic_name];
    rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);
    cb(extracted_serialized_msg);
  }
}
}  // namespace ruvu_mcl
