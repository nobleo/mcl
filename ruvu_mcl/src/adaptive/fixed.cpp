// Copyright 2021 RUVU Robotics B.V.

#include "./fixed.hpp"

#include "../config.hpp"
#include "../particle_filter.hpp"
#include "rclcpp/logging.hpp"

constexpr auto name = "fixed";

namespace ruvu_mcl
{
Fixed::Fixed(const Config & config) : max_particles_(config.max_particles)
{
  RCLCPP_INFO(rclcpp::get_logger(name), "using a fixed number of particles");
}

int Fixed::calc_needed_particles(const ParticleFilter & pf) const { return max_particles_; }
}  // namespace ruvu_mcl
