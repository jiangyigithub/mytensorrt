/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include <radar_locations_decoder_gen5_ros/radar_locations_decoder_gen5_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); 
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("radar_locations_decoder_gen5_ros");

  try {
    radar_locations_decoder_gen5_ros::RadarLocationsDecoderNode radar_locations_decoder_exec(node);
    radar_locations_decoder_exec.run();
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(node->get_logger(), "Terminating %s: %s", argv[0], e.what());
  }
}
