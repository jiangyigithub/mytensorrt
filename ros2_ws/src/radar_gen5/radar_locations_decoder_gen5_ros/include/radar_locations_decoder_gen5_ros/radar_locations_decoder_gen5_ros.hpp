/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

#include <radar_gen5_msgs/msg/location_interface.hpp>
#include <radar_msgs/msg/radar_rob2.hpp>
#include <radar_locations_decoder_gen5/radar_location_gen5.hpp>
#include "radar_locations_decoder_gen5_ros/optional_location_interface.hpp"
// #include <radar_ros_time_converter/radar_ros_time_converter_interface.hpp>


namespace radar_locations_decoder_gen5_ros
{
class RadarLocationsDecoderRos
{
public:
  // struct Parameters
  // {
  //   std::vector<std::string> device_type_names;
  // };

  RadarLocationsDecoderRos(
      std::shared_ptr<rclcpp::Node> node/*,
      Parameters params, radar_ros_time_converter::RadarRosTimeConverterInterface& time_converter*/);

  OptionalLocationInterface decodeLocationInterfaceMsgFromRobContainer(
      const radar_msgs::msg::RadarROB2& received_rob2) const;

  // bool isDeviceTypeSupported(const std::string& device_type) const;

  // Parameters getParamsFromNode();
  // Parameters getParamsFromYamlFile(const std::string& file_path);

private:
  //Parameters params_;
  std::shared_ptr<rclcpp::Node> node_;
  // radar_ros_time_converter::RadarRosTimeConverterInterface& time_converter_;
};
}  // namespace radar_locations_decoder_gen5_ros
