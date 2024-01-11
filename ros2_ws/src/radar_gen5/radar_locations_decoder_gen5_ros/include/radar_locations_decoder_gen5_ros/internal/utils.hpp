/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <string>
#include <vector>

// #include <ros/node_handle.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <radar_locations_decoder_gen5/radar_location_gen5.hpp>
// #include <radar_ros_time_converter/radar_ros_time_converter_interface.hpp>
#include <radar_msgs/msg/radar_rob2.hpp>
#include <radar_msgs/msg/rob_data.hpp>

#include "radar_locations_decoder_gen5_ros/optional_location_interface.hpp"
#include "radar_locations_decoder_gen5_ros/internal/device_information.hpp"

namespace radar_locations_decoder_gen5_ros
{
namespace internal
{
template <typename T>
T getParam(const std::string& param_key, std::shared_ptr<rclcpp::Node> node);

template <typename T>
T getParamFromYAML(const std::string& param_key, const YAML::Node& config);

OptionalLocationInterface convertLocationRobToLocationInterfaceMsg(
    const radar_msgs::msg::ROBData& rob,
    const DeviceInformation& device_information,
    // radar_ros_time_converter::RadarRosTimeConverterInterface& time_converter,
    const rclcpp::Time& rob_container_stamp,
    const std::shared_ptr<rclcpp::Node> node);

radar_gen5_msgs::msg::LocationInterface createLocationInterfaceMessage(
    const radar_locations_decoder_gen5::RadarLocationDataGen5& radar_location,
    const rclcpp::Time& time_stamp,
    const std::string& device_position,
    const std::shared_ptr<rclcpp::Node> node);

DeviceInformation extractDeviceInformationFromRobContainer(
    const radar_msgs::msg::RadarROB2& rob_container);

std::vector<radar_msgs::msg::ROBData> findLocationRobsInRobContainer(
    const radar_msgs::msg::RadarROB2& rob_container);

bool isRobOfLocationType(const radar_msgs::msg::ROBData& rob);

OptionalLocationInterface makeOptionalLocationInterface(radar_gen5_msgs::msg::LocationInterface msg);
OptionalLocationInterface makeOptionalLocationInterfaceEmpty();

std::string writeMetadataToString(const radar_msgs::msg::ROBData& rob);
std::string writeMetadataToString(const radar_msgs::msg::ROBData& rob, const DeviceInformation& device_information);

}  // namespace internal
}  // namespace radar_locations_decoder_gen5_ros

#include "../impl/utils_impl.hpp"
