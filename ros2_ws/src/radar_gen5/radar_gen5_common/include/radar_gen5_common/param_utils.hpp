/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */
#pragma once

#include "ros/node_handle.h"

namespace radar_gen5_common
{
template <typename T>
T getParam(const std::string& param_key, const ros::NodeHandle& nh)
{
  T param;
  if (!nh.getParam(param_key, param)) {
    std::string msg = "ROS Parameter '" + nh.getNamespace() + "/" + param_key + "' is not defined!";
    throw std::out_of_range(msg);
  }
  return param;
}

template <>
inline ros::Duration getParam(const std::string& param_key, const ros::NodeHandle& nh)
{
  return ros::Duration(getParam<double>(param_key, nh));
}
}  // namespace radar_gen5_common
