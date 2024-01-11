/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "radar_locations_decoder_gen5_ros/internal/utils.hpp"

#include <radar_gen5_common/clock_definition.hpp>
#include <radar_gen5_common/ticked_timestamp.hpp>
#include <radar_locations_decoder_gen5/decoder_exception.hpp>
#include <radar_locations_decoder_gen5/radar_locations_decoder.hpp>

namespace radar_locations_decoder_gen5_ros
{
namespace
{
// rclcpp::Time convertEcuTickCountToRosTime(
//     const DeviceInformation& device_information,
//     std::uint32_t ecu_ticks,
//     const rclcpp::Time& rob_ros_timestamp/*,
//     radar_ros_time_converter::RadarRosTimeConverterInterface& time_converter*/)
// {
//   radar_gen5_common::TickedTimestamp ecu_time{
//       ecu_ticks, radar_gen5_common::clocks::Base2Power16ClockWithMicrosecondsOverflow};
//   const auto sensor_id = device_information.position;
//   return time_converter.convertEcuTimeToRosTime(
//       sensor_id, ecu_time, rob_ros_timestamp, radar_gen5_common::DecoderType::LOCATIONS);
// }
}  // namespace

namespace internal
{
std::vector<radar_msgs::msg::ROBData> findLocationRobsInRobContainer(
    const radar_msgs::msg::RadarROB2& rob_container)
{
  std::vector<radar_msgs::msg::ROBData> location_robs;
  for (auto rob : rob_container.robs) {
    if (isRobOfLocationType(rob)) {
      location_robs.push_back(rob);
    }
  }
  return location_robs;
}

OptionalLocationInterface convertLocationRobToLocationInterfaceMsg(
    const radar_msgs::msg::ROBData& rob,
    const DeviceInformation& device_information,
    // radar_ros_time_converter::RadarRosTimeConverterInterface& time_converter,
    const rclcpp::Time& rob_container_stamp,
    const std::shared_ptr<rclcpp::Node> node)
{
  RCLCPP_DEBUG_STREAM(node->get_logger(),
      "Radar Decoder: Check ROB data. " << writeMetadataToString(rob, device_information));

  if (!isRobOfLocationType(rob)) {
    throw std::logic_error(
        "convertLocationRobToLocationMsg called with ROB that is not of location type.");
  }

  if (!rob.valid) {
    RCLCPP_WARN_STREAM(node->get_logger(),
        "Radar Decoder: ROB is not valid. " << writeMetadataToString(rob, device_information));
    return makeOptionalLocationInterfaceEmpty();
  }

  if (rob.data.size() == 0) {
    RCLCPP_WARN_STREAM(node->get_logger(),
        "Radar Decoder: Empty data array received. "
        << writeMetadataToString(rob, device_information));
    return makeOptionalLocationInterfaceEmpty();
  }

  RCLCPP_DEBUG_STREAM(node->get_logger(),
      "Radar Decoder: Decoding ROB. " << writeMetadataToString(rob, device_information));

  radar_locations_decoder_gen5::RadarDecoder radar_decoder;
  try {
    const auto radar_location =
        radar_decoder.decodeROBs(device_information.software_version, &rob.data[0]);

    // Skipping time conversion architecture for first step of migration to ROS2 - which anyway was not used by default.
    // rclcpp::Time ros_time = convertEcuTickCountToRosTime(
    //     device_information,
    //     radar_location.location_list.abs_meas_time,
    //     rob_container_stamp,
    //     time_converter);
    rclcpp::Time ros_time = rob_container_stamp;

    const auto msg =
        createLocationInterfaceMessage(radar_location, ros_time, device_information.position, node);
    return makeOptionalLocationInterface(msg);
  } catch (radar_locations_decoder_gen5::UnknownSoftwareVersionException& exception) {
    throw;
  } catch (radar_locations_decoder_gen5::DecoderException& exception) {
    RCLCPP_ERROR(node->get_logger(), "RadarLocationsDecoderRos: Exception: %s", exception.what());
    
    return makeOptionalLocationInterfaceEmpty();
  }
}

bool isRobOfLocationType(const radar_msgs::msg::ROBData& rob)
{
  const std::string ROB_NAME_LOCATION_1 = "_mempool_dspRunnable_m_dspLocationList_out";
  const std::string ROB_NAME_LOCATION_2 =
      "_g_ENS_R_DspRunnable_DspRunnable_m_LocationInterface_out_local.TMemPool";
  return ((rob.name == ROB_NAME_LOCATION_1) || (rob.name == ROB_NAME_LOCATION_2));
}

DeviceInformation extractDeviceInformationFromRobContainer(
    const radar_msgs::msg::RadarROB2& rob_container)
{
  DeviceInformation metadata;
  metadata.software_version = rob_container.sw_revision;
  metadata.type = rob_container.device_type;
  metadata.position = rob_container.device_position;
  return metadata;
}

radar_gen5_msgs::msg::LocationInterface createLocationInterfaceMessage(
    const radar_locations_decoder_gen5::RadarLocationDataGen5& radar_location,
    const rclcpp::Time& time_stamp,
    const std::string& device_position,
    const std::shared_ptr<rclcpp::Node> node)
{
  RCLCPP_DEBUG_STREAM(node->get_logger(),"radar_locations_decoder_gen5_ros: start processLocation");

  // check radar status
  const radar_locations_decoder_gen5::RadarSensingStateGen5& sensing_state =
      radar_location.sensing_state;

  // fill ROS message
  const radar_locations_decoder_gen5::RadarLocationListGen5& location_list =
      radar_location.location_list;

  radar_gen5_msgs::msg::LocationInterface location_interface_msg;

  // fill header
//   location_interface_msg.header.seq = radar_location.sequence_number;
  location_interface_msg.header.stamp = time_stamp;
  location_interface_msg.header.frame_id = device_position;
  location_interface_msg.radar_ecu_timestamp =
      location_list.abs_meas_time /
      radar_locations_decoder_gen5::RadarDecoder::NORMALIZATION_FACTOR_ECU_TIMESTAMP;

  // fill status
  location_interface_msg.sensing_state.operation_mode = sensing_state.op_mode;
  location_interface_msg.sensing_state.measurement_state = sensing_state.data_measured;
  location_interface_msg.sensing_state.thermal_degredation_factor =
      vfc::float32_t(sensing_state.fac_thermal_degradation);

  // TODO: fill blindness interference_status (if needed)

  // fill field of view
  for (int i = 0;
       i < location_interface_msg.sensing_state.field_of_view.maximum_fov_range_azimuth.size();
       ++i)
    location_interface_msg.sensing_state.field_of_view.maximum_fov_range_azimuth[i] =
        sensing_state.field_of_view.maximum_fov_range_azimuth[i];
  for (int i = 0; i < location_interface_msg.sensing_state.field_of_view.fov_azimuth_angles.size();
       ++i)
    location_interface_msg.sensing_state.field_of_view.fov_azimuth_angles[i] =
        sensing_state.field_of_view.fov_azimuth_angles[i];
  for (int i = 0;
       i < location_interface_msg.sensing_state.field_of_view.range_scaling_elevation.size();
       ++i)
    location_interface_msg.sensing_state.field_of_view.range_scaling_elevation[i] =
        sensing_state.field_of_view.range_scaling_elevation[i];
  for (int i = 0;
       i < location_interface_msg.sensing_state.field_of_view.fov_elevation_angles.size();
       ++i)
    location_interface_msg.sensing_state.field_of_view.fov_elevation_angles[i] =
        sensing_state.field_of_view.fov_elevation_angles[i];

  // fill misalignment
  location_interface_msg.sensing_state.misalignment.estimation_status =
      sensing_state.misalignment.estimation_status;
  location_interface_msg.sensing_state.misalignment.azimuth_angle_misalignment =
      sensing_state.misalignment.azimuth_angle_misalignment;
  location_interface_msg.sensing_state.misalignment.azimuth_angle_misalignment_variance =
      sensing_state.misalignment.azimuth_angle_misalignment_variance;
  location_interface_msg.sensing_state.misalignment.elevation_angle_misalignment =
      sensing_state.misalignment.elevation_angle_misalignment;
  location_interface_msg.sensing_state.misalignment.elevation_angle_misalignment_variance =
      sensing_state.misalignment.elevation_angle_misalignment_variance;
  // fill modulation performance
  location_interface_msg.sensing_state.modulation_performance
      .active_detection_measurement_program_id =
      sensing_state.modulation_performance.active_detection_measurement_program_id;
  location_interface_msg.sensing_state.modulation_performance.active_modulation_id =
      sensing_state.modulation_performance.active_modulation_id;
  location_interface_msg.sensing_state.modulation_performance.fov_distance_range_scaling_factor =
      sensing_state.modulation_performance.fov_distance_range_scaling_factor;
  location_interface_msg.sensing_state.modulation_performance.distance_precision =
      sensing_state.modulation_performance.distance_precision;
  location_interface_msg.sensing_state.modulation_performance.distance_seperability =
      sensing_state.modulation_performance.distance_seperability;
  location_interface_msg.sensing_state.modulation_performance.distance_minimum_value =
      sensing_state.modulation_performance.distance_minimum_value;
  location_interface_msg.sensing_state.modulation_performance.distance_maximum_value =
      sensing_state.modulation_performance.distance_maximum_value;
  location_interface_msg.sensing_state.modulation_performance.velocity_precision =
      sensing_state.modulation_performance.velocity_precision;
  location_interface_msg.sensing_state.modulation_performance.velocity_seperability =
      sensing_state.modulation_performance.velocity_seperability;
  location_interface_msg.sensing_state.modulation_performance.velocity_minimum_value =
      sensing_state.modulation_performance.velocity_minimum_value;
  location_interface_msg.sensing_state.modulation_performance.velocity_maximum_value =
      sensing_state.modulation_performance.velocity_maximum_value;
  location_interface_msg.sensing_state.modulation_performance.distance_velocity_covariance =
      sensing_state.modulation_performance.distance_velocity_covariance;

  // fill locations
  for (int locIdx = 0; locIdx < location_list.number_of_locations; locIdx++) {
    radar_gen5_msgs::msg::Location location_msg;
    const radar_locations_decoder_gen5::RadarLocationItemGen5& loc_item =
        location_list.item[locIdx];

    location_msg.radial_distance = loc_item.radial_distance;
    location_msg.radial_distance_variance = loc_item.radial_distance_var;
    location_msg.radial_distance_spread = loc_item.radial_distance_spread;  // not used anymore

    location_msg.radial_velocity = loc_item.relative_radial_velocity;
    location_msg.radial_velocity_variance = loc_item.relative_radial_velocity_var;
    location_msg.radial_velocity_spread =
        loc_item.relative_radial_velocity_spread;  // not used anymore

    location_msg.radial_distance_velocity_covariance = loc_item.radial_distance_velocity_cov;
    location_msg.radial_distance_velocity_quality = loc_item.radial_distance_velocity_quality;
    location_msg.radial_distance_velocity_spread_orientation =
        loc_item.distance_velocity_spread_orientation;  // not used anymore

    location_msg.azimuth_angle = loc_item.azimuth_angle;  // azimuth angle: theta [rad]
    location_msg.azimuth_angle_quality = loc_item.azimuth_angle_quality;
    location_msg.azimuth_angle_variance = loc_item.azimuth_angle_var;
    location_msg.elevation_angle = loc_item.elevation_angle;  // elevation angle: phi [rad]
    location_msg.elevation_angle_quality = loc_item.elevation_angle_quality;
    location_msg.elevation_angle_variance = loc_item.elevation_angle_var;

    location_msg.rcs = loc_item.rcs;    // [dBm2]
    location_msg.rssi = loc_item.rssi;  // [-]

    location_msg.measurement_status = loc_item.measurement_status;
    // note: the LOC_STANDING flag in measStatus is no longer supported by CC-DA, do not use it.

    location_interface_msg.raw_locations.push_back(location_msg);
  }
  return location_interface_msg;
}

OptionalLocationInterface makeOptionalLocationInterface(radar_gen5_msgs::msg::LocationInterface msg)
{
  return {true, msg};
}

OptionalLocationInterface makeOptionalLocationInterfaceEmpty()
{
//   return {false, {}};
   return {false, radar_gen5_msgs::msg::LocationInterface() };
}

std::string writeMetadataToString(
    const radar_msgs::msg::ROBData& rob, const DeviceInformation& device_information)
{
  std::stringstream ss;
  ss << writeMetadataToString(rob) << ", " << device_information;
  return ss.str();
}

std::string writeMetadataToString(const radar_msgs::msg::ROBData& rob)
{
  std::stringstream ss;
  ss << "packet name: " << rob.name;
  return ss.str();
}
}  // namespace internal
}  // namespace radar_locations_decoder_gen5_ros
