// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "../config.hpp"
#include "./laser.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"

// forward declare
namespace visualization_msgs::msg
{
ROS_DECLARE_MESSAGE(Marker)
}

namespace ruvu_mcl_msgs::msg
{
ROS_DECLARE_MESSAGE(ParticleStatistics)
}

namespace ruvu_mcl
{
// forward declare
struct OccupancyMap;

/**
 * @brief Implements the beam range finder model from Probablistic Robotics
 */
class BeamModel : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  BeamModel(
    rclcpp::Node::SharedPtr nh, const BeamModelConfig & config,
    const std::shared_ptr<const OccupancyMap> & map);

  void sensor_update(ParticleFilter * pf, const LaserData & data) override;

private:
  const BeamModelConfig parameters_;
  const std::shared_ptr<const OccupancyMap> map_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> debug_pub_;
  std::shared_ptr<rclcpp::Publisher<ruvu_mcl_msgs::msg::ParticleStatistics>> statistics_pub_;
};
}  // namespace ruvu_mcl
