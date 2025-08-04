// Copyright 2021 RUVU Robotics B.V.

#include "./bag_buffer.hpp"

#include <chrono>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <string>
#include <vector>

#include "tf2_msgs/msg/tf_message.hpp"

namespace ruvu_mcl
{
std::shared_ptr<const tf2::BufferCore> create_bag_buffer(rosbag2_cpp::Reader & bag)
{
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = {"/tf", "/tf_static"};
  bag.set_filter(storage_filter);

  std::vector<geometry_msgs::msg::TransformStamped> tfs;
  std::vector<geometry_msgs::msg::TransformStamped> static_tfs;
  while (rclcpp::ok() && bag.has_next()) {
    auto msg = bag.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);
    tf2_msgs::msg::TFMessage tf_message;
    rclcpp::Serialization<decltype(tf_message)> serialization;
    serialization.deserialize_message(&extracted_serialized_msg, &tf_message);

    for (const auto & tf : tf_message.transforms) {
      if (msg->topic_name == "/tf") {
        tfs.push_back(tf);
      } else if (msg->topic_name == "/tf_static") {
        static_tfs.push_back(tf);
      } else {
        assert(false && "Unexpected topic name in bag file, expected /tf or /tf_static");
      }
    }
  }

  if (tfs.size() < 2) {
    throw std::runtime_error("Not enough transforms in bag file, at least 2 are required");
  }
  rclcpp::Time first = tfs.front().header.stamp;
  rclcpp::Time last = tfs.back().header.stamp;

  std::chrono::nanoseconds duration{last.nanoseconds() - first.nanoseconds()};
  auto buffer = std::make_shared<tf2::BufferCore>(duration);

  for (const auto & tf : tfs) {
    buffer->setTransform(tf, "bagfile", tf.header.frame_id == "tf_static");
  }

  return buffer;
}
}  // namespace ruvu_mcl
