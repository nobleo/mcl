// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "../config.hpp"
#include "../message_forward.hpp"
#include "./landmark.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "ruvu_mcl_msgs/msg/particle_statistics.hpp"
#include "visualization_msgs/msg/marker.hpp"

// forward declare
namespace ruvu_mcl_msgs::msg
{
ROS_DECLARE_MESSAGE(LandmarkList)
}

namespace ruvu_mcl
{
/**
 * @brief Impelements the likelihood field range finder model from Probablistic Robotics for landmark measurments
 */
class LandmarkLikelihoodFieldModel : public LandmarkModel
{
public:
  /*
   * @brief LikelihoodFieldModel constructor
   */
  LandmarkLikelihoodFieldModel(
    rclcpp::Node::SharedPtr nh, const LandmarkLikelihoodFieldModelConfig & config,
    const LandmarkList & landmarks);

  void sensor_update(ParticleFilter * pf, const LandmarkList & data) override;

private:
  const LandmarkLikelihoodFieldModelConfig config_;
  const LandmarkList landmarks_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_pub_;
  rclcpp::Publisher<ruvu_mcl_msgs::msg::ParticleStatistics>::SharedPtr statistics_pub_;
};
}  // namespace ruvu_mcl
