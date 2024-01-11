/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include <rclcpp/rclcpp.hpp>
#include <radar_locations_to_pointcloud_converter_ros/radar_locations_to_pointcloud_converter_ros.hpp>

int main(int argc, char **argv) {

  std::cout << "rclcpp::init" << std::endl;
  rclcpp::init(argc, argv);
  auto radar_converter_exec = std::make_shared<radar_locations_to_pointcloud_converter_ros::CRadarLocationsToPointcloudConverterRos>();

  rclcpp::spin(radar_converter_exec);
  rclcpp::shutdown();

  return 0;
}
