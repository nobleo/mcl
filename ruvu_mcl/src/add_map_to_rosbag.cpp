#include <iostream>
#include <memory>
#include <string>

#include "nav2_map_server/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <map.yaml> <input_bag> <output_bag>" << std::endl;
    return EXIT_FAILURE;
  }

  std::string map_yaml = argv[1];
  std::string input_bag = argv[2];
  std::string output_bag = argv[3];

  // --- Load map from YAML ---
  nav_msgs::msg::OccupancyGrid map_msg;
  auto result = nav2_map_server::loadMapFromYaml(map_yaml, map_msg);
  switch (result) {
    case nav2_map_server::MAP_DOES_NOT_EXIST:
      std::cerr << "Map file does not exist: " << map_yaml << std::endl;
      return EXIT_FAILURE;
    case nav2_map_server::INVALID_MAP_METADATA:
      std::cerr << "Invalid map metadata in file: " << map_yaml << std::endl;
      return EXIT_FAILURE;
    case nav2_map_server::INVALID_MAP_DATA:
      std::cerr << "Invalid map data in file: " << map_yaml << std::endl;
      return EXIT_FAILURE;
    case nav2_map_server::LOAD_MAP_SUCCESS:
      std::cout << "Map loaded successfully from: " << map_yaml << std::endl;
      break;
  }

  // Create the reader
  rosbag2_storage::StorageOptions reader_storage_options;
  reader_storage_options.uri = input_bag;
  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(reader_storage_options);
  reader->open(reader_storage_options);

  // Create the writer
  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  rosbag2_storage::StorageOptions writer_storage_options;
  writer_storage_options.uri = output_bag;
  writer->open(writer_storage_options);

  // --- Ensure all topics are created in the output bag ---
  const auto topics = reader->get_all_topics_and_types();
  for (const auto & topic : topics) {
    writer->create_topic(topic);
  }
  rosbag2_storage::TopicMetadata map_topic_metadata;
  map_topic_metadata.name = "/map";
  map_topic_metadata.type = "nav_msgs/msg/OccupancyGrid";
  map_topic_metadata.serialization_format = "cdr";
  map_topic_metadata.offered_qos_profiles = {rclcpp::QoS{1}.transient_local()};
  writer->create_topic(map_topic_metadata);

  // Write the map first
  rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  writer->write(map_msg, "/map", t0);

  // Then copy the rest of the bagfile
  while (reader->has_next()) {
    auto bag_msg = reader->read_next();
    writer->write(bag_msg);
  }

  std::cout << "New bag written to: " << output_bag << std::endl;

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
