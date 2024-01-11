/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef RADAR_MANAGER_PARAMETERS_HPP_
#define RADAR_MANAGER_PARAMETERS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"


namespace radar_manager
{
struct RadarManagerParameters
{
  void getParamsFromNodeHandle(const std::shared_ptr<rclcpp::Node> node_);

//   template <typename T>
//   void getParam(T& param, const std::string& param_key, const std::shared_ptr<rclcpp::Node> node_)
//   {
//     if (!nh.getParam(param_key, param)) {
//       ROS_ERROR_STREAM("Parameter '" << param_key << "' is not defined!");
//       std::exit(EXIT_FAILURE);
//     }
//   }

  std::string publisher_topic_robs;
  int msg_queue_size;
  double desired_freq_min;
  double desired_freq_max;
  double freq_tol;
};
} /* namespace radar_manager */

#endif
