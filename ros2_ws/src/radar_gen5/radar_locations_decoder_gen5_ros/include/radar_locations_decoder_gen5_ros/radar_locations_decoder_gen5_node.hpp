/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <memory>
#include <string>
#include <map>

// #include <ros/node_handle.h>
// #include <ros/timer.h>
// #include <rclcpp/rclcpp.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <radar_msgs/msg/radar_rob2.h>
// #include <radar_ros_time_converter/radar_ros_time_converter_interface.hpp>
#include "radar_locations_decoder_gen5_ros/radar_locations_decoder_gen5_ros.hpp"


namespace radar_locations_decoder_gen5_ros
{
class RadarLocationsDecoderNode
{
public:
  struct Parameters
  {
    std::string subscriber_topic_robs;
    int subscriber_msg_queue_size;
    double desired_freq_locations;
    double freq_tol_locations;

    std::string publisher_topic_locations;
    int publisher_msg_queue_size;
    bool publisher_locations_to_individual_topics_per_sensor;
  };

  RadarLocationsDecoderNode(std::shared_ptr<rclcpp::Node> node);
  void run();
  void receiveRobContainer(const radar_msgs::msg::RadarROB2::SharedPtr received_rob2);

  Parameters getParamsFromNode();
  Parameters getParametersFromYamlFile(const std::string& file_path);

private:
  // std::unique_ptr<radar_ros_time_converter::RadarRosTimeConverterInterface>
  //     radar_ros_time_converter_;
  radar_locations_decoder_gen5_ros::RadarLocationsDecoderRos radar_locations_decoder_;

  rclcpp::Subscription<radar_msgs::msg::RadarROB2>::SharedPtr radar_rob_sub2_;
  std::map<std::string, std::shared_ptr<rclcpp::Publisher<radar_gen5_msgs::msg::LocationInterface>>> map_radar_gen5_publisher_;

  // Node handle is needed for on-demand creation of publishers
  std::shared_ptr<rclcpp::Node> node_;

  diagnostic_updater::Updater diag_updater_locations_;
  rclcpp::TimerBase::SharedPtr diag_updater_locations_timer_;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> diag_radar_decoder_locations_;

  Parameters params_;

  //lnl, timer
  rclcpp::Time radar_decoder_in_t{0};
  rclcpp::Time radar_decoder_out_t{0};
  rclcpp::Duration radar_decoder_d_t{0};
  uint32_t radar_decoder_t_origin{0};
};
}
