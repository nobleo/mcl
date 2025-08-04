// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "../config.hpp"
#include "./laser.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "ruvu_mcl_msgs/msg/particle_statistics.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace ruvu_mcl
{
// forward declare
struct DistanceMap;

/**
 * @brief Impelements the likelihood field range finder model from Probablistic Robotics
 */
class LikelihoodFieldModel : public Laser
{
public:
  /*
   * @brief LikelihoodFieldModel constructor
   */
  LikelihoodFieldModel(
    rclcpp::Node::SharedPtr nh, const LikelihoodFieldModelConfig & config,
    const std::shared_ptr<const DistanceMap> & map);

  void sensor_update(ParticleFilter * pf, const LaserData & data) override;

private:
  const LikelihoodFieldModelConfig config_;
  const std::shared_ptr<const DistanceMap> map_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_pub_;
  rclcpp::Publisher<ruvu_mcl_msgs::msg::ParticleStatistics>::SharedPtr statistics_pub_;
};
}  // namespace ruvu_mcl
