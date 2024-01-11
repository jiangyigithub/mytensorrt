/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "radar_manager_ros/radar_manager_parameters.hpp"

namespace radar_manager
{
void RadarManagerParameters::getParamsFromNodeHandle(const std::shared_ptr<rclcpp::Node> node_)
{
  std::cout << "[RadarManagerParameters::getParamsFromNodeHandle()]" << std::endl;

  std::cout << "[RadarManagerParameters::getParamsFromNodeHandle()] reading parameter publisher_topic_robs" << std::endl;
  publisher_topic_robs = node_->get_parameter("publisher_topic_robs").as_string();
  std::cout << "[RadarManagerParameters::getParamsFromNodeHandle()] reading parameter msg_queue_size" << std::endl;
  msg_queue_size = node_->get_parameter("msg_queue_size").as_int();
  std::cout << "[RadarManagerParameters::getParamsFromNodeHandle()] reading parameter desired_freq_min" << std::endl;
  desired_freq_min = node_->get_parameter("desired_freq_min").as_double();
  std::cout << "[RadarManagerParameters::getParamsFromNodeHandle()] reading parameter desired_freq_max" << std::endl;
  desired_freq_max = node_->get_parameter("desired_freq_max").as_double();
  std::cout << "[RadarManagerParameters::getParamsFromNodeHandle()] reading parameter freq_tol" << std::endl;
  freq_tol = node_->get_parameter("freq_tol").as_double();

  std::cout << "[RadarManagerParameters::getParamsFromNodeHandle()]"
                    << " publisher_topic_robs: " << publisher_topic_robs
                    << ", msg_queue_size: " << msg_queue_size
                    << ", desired_freq_min: " << desired_freq_min
                    << ", desired_freq_max: " << desired_freq_max
                    << ", freq_tol: " << freq_tol 
                    << std::endl;

  std::cout << "[RadarManagerParameters::getParamsFromNodeHandle()] Done" << std::endl;
}
} /* namespace radar_manager */
