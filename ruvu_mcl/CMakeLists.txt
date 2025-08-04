cmake_minimum_required(VERSION 3.8)
project(ruvu_mcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ruvu_mcl_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams filesystem)
find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_map_server REQUIRED)

###########
## Build ##
###########

generate_parameter_library(
  parameters
  src/parameters.yaml
)

add_library(${PROJECT_NAME}
  src/adaptive/fixed.cpp
  src/adaptive/kld_sampling.cpp
  src/adaptive/split_and_merge.cpp
  src/adaptive/utils.cpp
  src/cloud_publisher.cpp
  src/config.cpp
  src/map.cpp
  src/mcl.cpp
  src/mcl_ros.cpp
  src/motion_models/differential_motion_model.cpp
  src/motion_models/motion_model.cpp
#  src/node.cpp
  src/offline/bag_buffer.cpp
  src/offline/bag_player.cpp
  src/particle_filter.cpp
  src/resamplers/low_variance.cpp
  src/resamplers/resampler.cpp
  src/rng.cpp
  src/sensor_models/beam_model.cpp
  src/sensor_models/gaussian_landmark_model.cpp
  src/sensor_models/landmark.cpp
  src/sensor_models/landmark_likelihood_field_model.cpp
  src/sensor_models/laser.cpp
  src/sensor_models/likelihood_field_model.cpp
  src/mcl_component.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  tf2::tf2
  angles::angles
  rosbag2_transport::rosbag2_transport
  parameters
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${ruvu_mcl_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  rclcpp_components::component
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "ruvu_mcl::MclComponent"
  EXECUTABLE mcl
)

add_executable(offline src/offline.cpp)
target_link_libraries(offline PRIVATE
  ${PROJECT_NAME}
)

add_executable(add_map_to_rosbag src/add_map_to_rosbag.cpp)
target_include_directories(add_map_to_rosbag
  SYSTEM PRIVATE ${nav2_map_server_INCLUDE_DIRS}
)
target_link_libraries(add_map_to_rosbag PRIVATE
  ${PROJECT_NAME}
  ${nav2_map_server_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME} parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS offline add_map_to_rosbag
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(FILTERS -build/include_order)

  ament_add_gtest(unittests SKIP_LINKING_MAIN_LIBRARIES
    test/test_adaptive_utils.cpp
    test/test_differential_motion_model.cpp
    test/test_distance_map.cpp
    test/test_gaussian_landmark_model.cpp
    test/test_map.cpp
    test/test_occupancy_map.cpp
    test/test_particle_filter.cpp
    test/test_split_and_merge.cpp
    test/main.cpp
  )
  target_link_libraries(unittests ${PROJECT_NAME})

  add_executable(plot_motion_models test/plot_motion_models.cpp)
  target_link_libraries(plot_motion_models PRIVATE ${PROJECT_NAME} Boost::iostreams Boost::filesystem)

  add_executable(plot_resampling test/plot_resampling.cpp)
  target_link_libraries(plot_resampling PRIVATE ${PROJECT_NAME} Boost::iostreams Boost::filesystem)

  add_executable(render_scan test/render_scan.cpp)
  target_include_directories(render_scan SYSTEM PRIVATE ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})
  target_link_libraries(render_scan PRIVATE ${GRAPHICSMAGICKCPP_LIBRARIES})
  target_link_libraries(render_scan PRIVATE ${PROJECT_NAME})
endif()

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
