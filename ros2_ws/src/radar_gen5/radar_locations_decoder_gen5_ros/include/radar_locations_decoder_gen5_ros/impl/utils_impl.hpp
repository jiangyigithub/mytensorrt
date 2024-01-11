/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <string>
// #include <ros/node_handle.h>
#include <yaml-cpp/yaml.h>

#include "radar_locations_decoder_gen5_ros/internal/ros_decoder_exception.hpp"

namespace radar_locations_decoder_gen5_ros
{
namespace utils
{
template <typename T>
T getParam(std::shared_ptr<rclcpp::Node> node, const std::string& param_key)
{
  T param_value;
  rclcpp::Parameter parameter; 

  if (!node->has_parameter(param_key))
      throw RosDecoderException("Parameter '" + param_key + "' does not exist");

  if (!node->get_parameter(param_key, parameter))
      throw RosDecoderException("Parameter '" + param_key + "' could not be retrieved or is not set");

  RCLCPP_INFO_STREAM(node->get_logger(), "Parameter: " << param_key << ", value_to_string: " << parameter.value_to_string());

  param_value = parameter.get_value<T>();
  return param_value;
}

template <typename T>
T getParamOrDefault(std::shared_ptr<rclcpp::Node> node, const std::string& param_key, const T& default_value)
{
  T param_value;

  if (!node->has_parameter(param_key))
    node->declare_parameter(param_key);

  if (node->get_parameter(param_key).get_parameter_value() == rclcpp::ParameterValue() ) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Parameter '" + param_key + "' is not defined, using default value:" << default_value);
    param_value = default_value;
  }
  else
    param_value = node->get_parameter(param_key).get_value<T>();

  return param_value;
}

template <typename T>
T getParamFromYAML(const std::string& param_key, const YAML::Node& config)
{
  if (config[param_key]) {
    return config[param_key].as<T>();
  } else {
    throw RosDecoderException(
        "Parameter '" + param_key + "' does not exist in the provided yaml file");
  }
}

template <typename T>
T getParamFromYAMLOrDefault(const std::shared_ptr<rclcpp::Node> node, const std::string& param_key, const YAML::Node& config, const T& default_value)
{
  if (config[param_key]) {
    return config[param_key].as<T>();
  } else {
    RCLCPP_WARN_STREAM(node->get_logger(), "Parameter '" + param_key + "' does not exist in the provided yaml file, using default value:" << default_value);
    return default_value;
  }
}

}  // namespace utils
}  // namespace radar_locations_decoder_gen5_ros
