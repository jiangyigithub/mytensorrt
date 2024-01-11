/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "radar_locations_decoder_gen5_ros/radar_locations_decoder_gen5_ros.hpp"

#include <radar_locations_decoder_gen5/radar_locations_decoder.hpp>
// #include <ros/console.h>
//#include <ros/time.h>

#include "radar_locations_decoder_gen5_ros/internal/utils.hpp"

namespace radar_locations_decoder_gen5_ros
{
RadarLocationsDecoderRos::RadarLocationsDecoderRos(
    std::shared_ptr<rclcpp::Node> node
    //Parameters params,
     //, radar_ros_time_converter::RadarRosTimeConverterInterface& time_converter
) : node_(node)/*,
    params_(params), time_converter_(time_converter)*/
{
  // Todo Joachim no longer needed in ROS2?
  // if (!rclcpp::Time::isValid()) {
  //   rclcpp::Time::init();  // If the caller is not(yet) a ros node, or not(yet) using rclcpp::Time,
  //                       // initialize it here.
  // }
}

OptionalLocationInterface RadarLocationsDecoderRos::decodeLocationInterfaceMsgFromRobContainer(
    const radar_msgs::msg::RadarROB2& rob_container) const
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Radar Decoder: Start data processing for RadarROB v.2");

  auto device = internal::extractDeviceInformationFromRobContainer(rob_container);

  // if (!isDeviceTypeSupported(device.type)) {
  //   RCLCPP_WARN_STREAM(node_.get_logger(), "Radar Decoder Gen5: Unknown device ID: " << rob_container.device_type);
  //   return {};
  // };

  auto location_robs = internal::findLocationRobsInRobContainer(rob_container);

  if (location_robs.size() == 0) {
    return internal::makeOptionalLocationInterfaceEmpty();
  }
  if (location_robs.size() > 1) {
    RCLCPP_WARN_STREAM(node_->get_logger(),
        "Radar Decoder: Multiple Location ROBS in the rob container. Only first one will be used. "
        << device);
  }

  return internal::convertLocationRobToLocationInterfaceMsg(
      location_robs.front(), 
      device, 
      // time_converter_, 
      rob_container.header.stamp,
      node_);
}

// bool RadarLocationsDecoderRos::isDeviceTypeSupported(const std::string& device_type) const
// {
//   return std::find(
//              params_.device_type_names.begin(), params_.device_type_names.end(), device_type) !=
//          params_.device_type_names.end();
// }

// RadarLocationsDecoderRos::Parameters RadarLocationsDecoderRos::getParamsFromNode()
// {
//   Parameters params;
//   params.device_type_names = utils::getParam<std::vector<std::string>>("device_type_names", node_);
//   return params;
// }

// RadarLocationsDecoderRos::Parameters RadarLocationsDecoderRos::getParamsFromYamlFile(
//     const std::string& file_path)
// {
//   YAML::Node config = YAML::LoadFile(file_path);
//   Parameters params;
//   params.device_type_names =
//       utils::getParamFromYAML<std::vector<std::string>>("device_type_names", config);
//   return params;
// }
}  // namespace radar_locations_decoder_gen5_ros
