/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <string>
#include <sstream>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <radar_gen5_msgs/msg/location_interface.hpp>


namespace radar_locations_to_pointcloud_converter_ros 
{

class CRadarLocationsToPointcloudConverterRos : public rclcpp::Node
{
public:
  struct Parameters
  {
    std::vector<std::string> subscriber_topic_locations;
    std::vector<std::string> subscriber_frame_id;
    int subscriber_msg_queue_size;
    std::string publisher_topic_location_cloud_prefix;
    int publisher_msg_queue_size;
  };

  CRadarLocationsToPointcloudConverterRos();

private:
  void LocationReceiverCallback(const radar_gen5_msgs::msg::LocationInterface::SharedPtr locations_msg);
  Parameters getParamsFromNodeHandle();

  std::vector<rclcpp::Subscription<radar_gen5_msgs::msg::LocationInterface>::SharedPtr> radar_loc_subscr_;

  int num_ros_subscribers_;
  Parameters params_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> m_mapPublisherRadarLocationCloud;
};
} // namespace radar_locations_decoder_gen5_ros
