/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */


//#include <ros/time.h>
// #include <std_msgs/Header.h>
#include <yaml-cpp/yaml.h>

#include <radar_gen5_msgs/msg/location_interface.h>
#include <radar_locations_decoder_gen5/radar_location_list_gen5.hpp>
#include <radar_locations_decoder_gen5/radar_location_sensing_state_gen5.hpp>
#include <radar_locations_decoder_gen5/radar_locations_decoder.hpp>
// #include <radar_ros_time_converter/make_converter.hpp>

#include "radar_locations_decoder_gen5_ros/radar_locations_decoder_gen5_node.hpp"
#include "radar_locations_decoder_gen5_ros/internal/utils.hpp"

using std::placeholders::_1;


namespace radar_locations_decoder_gen5_ros
{
RadarLocationsDecoderNode::RadarLocationsDecoderNode(std::shared_ptr<rclcpp::Node> node)
  : node_(node),
    radar_locations_decoder_(node),
    diag_updater_locations_(node)
    // radar_ros_time_converter_(
    //     radar_ros_time_converter::makeConverter(ros::NodeHandle(nh, "radar_ros_time_converter"))),
   // radar_locations_decoder_(RadarLocationsDecoderRos::getParamsFromNode(node), 
   // radar_ros_time_converter_*/)  
{
  std::cout << "[RadarLocationsDecoderNode::RadarLocationsDecoderNode] Declaring all params..." << std::endl;

  node->declare_parameter("subscriber_topic_robs");
  node->declare_parameter("subscriber_msg_queue_size");
  node->declare_parameter("publisher_topic_locations");
  node->declare_parameter("desired_freq_locations");
  node->declare_parameter("freq_tol_locations");
  node->declare_parameter("publisher_msg_queue_size");
  node->declare_parameter("publisher_locations_to_individual_topics_per_sensor");

  std::cout << "[RadarLocationsDecoderNode::RadarLocationsDecoderNode] Reading params" << std::endl;
  params_ = getParamsFromNode();

  radar_rob_sub2_ = node->create_subscription<radar_msgs::msg::RadarROB2>(
      params_.subscriber_topic_robs,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&radar_locations_decoder_gen5_ros::RadarLocationsDecoderNode::receiveRobContainer, this, _1));

  // diagnostic updater of locations decoder
  diag_updater_locations_.setHardwareID("radar_locations_decoder_gen5");
  diag_radar_decoder_locations_ = std::make_unique<diagnostic_updater::TopicDiagnostic>(
      params_.publisher_topic_locations,
      diag_updater_locations_,
      diagnostic_updater::FrequencyStatusParam(
          &params_.desired_freq_locations,
          &params_.desired_freq_locations,
          params_.freq_tol_locations),
      diagnostic_updater::TimeStampStatusParam(-1, 1));

  diag_updater_locations_timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000), [this]{ diag_updater_locations_.force_update(); } );
}

RadarLocationsDecoderNode::Parameters RadarLocationsDecoderNode::getParamsFromNode()
{
  Parameters params;

  RCLCPP_INFO_STREAM(node_->get_logger(), "RadarLocationsDecoderNode::getParamsFromNode()");

  params.subscriber_topic_robs = utils::getParam<std::string>(node_, "subscriber_topic_robs");
  params.subscriber_msg_queue_size = utils::getParam<int>(node_, "subscriber_msg_queue_size");
  params.publisher_topic_locations = utils::getParam<std::string>(node_, "publisher_topic_locations");
  params.desired_freq_locations = utils::getParam<double>(node_, "desired_freq_locations");
  params.freq_tol_locations = utils::getParam<double>(node_, "freq_tol_locations");
  params.publisher_msg_queue_size = utils::getParam<int>(node_, "publisher_msg_queue_size");
  params.publisher_locations_to_individual_topics_per_sensor = utils::getParamOrDefault<int>(node_, "publisher_locations_to_individual_topics_per_sensor", 0);

  RCLCPP_INFO_STREAM(node_->get_logger(), "RadarLocationsDecoderNode::getParamsFromNode(): Successful");
  //lnl : test
  std::cout << "[RadarLocationsDecoderNode::RadarLocationsDecoderNode] Reading params" << params.subscriber_topic_robs <<std::endl;

  return params;
}

RadarLocationsDecoderNode::Parameters RadarLocationsDecoderNode::getParametersFromYamlFile(
    const std::string& file_path)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "RadarLocationsDecoderNode::getParametersFromYamlFile()");

  YAML::Node config = YAML::LoadFile(file_path);
  Parameters params;

  params.subscriber_topic_robs = utils::getParamFromYAML<std::string>("subscriber_topic_robs", config);
  params.subscriber_msg_queue_size = utils::getParamFromYAML<int>("subscriber_msg_queue_size", config);
  params.publisher_topic_locations = utils::getParamFromYAML<std::string>("publisher_topic_locations", config);
  params.desired_freq_locations = utils::getParamFromYAML<double>("desired_freq_locations", config);
  params.freq_tol_locations = utils::getParamFromYAML<double>("freq_tol_locations", config);
  params.publisher_msg_queue_size = utils::getParamFromYAML<int>("publisher_msg_queue_size", config);
  params.publisher_locations_to_individual_topics_per_sensor = utils::getParamFromYAMLOrDefault<int>(node_, "publisher_locations_to_individual_topics_per_sensor", config, false);

  RCLCPP_INFO_STREAM(node_->get_logger(), "RadarLocationsDecoderNode::getParametersFromYamlFile(): Successful");

  return params;
}

void RadarLocationsDecoderNode::receiveRobContainer(
    const radar_msgs::msg::RadarROB2::SharedPtr received_rob2)
{
  //lnl, timer
  const auto time_temp_in = node_->now();
  radar_decoder_in_t = time_temp_in;

    auto optional_location_msg = radar_locations_decoder_.decodeLocationInterfaceMsgFromRobContainer(*received_rob2);
  std::shared_ptr<rclcpp::Publisher<radar_gen5_msgs::msg::LocationInterface>> radar_gen5_publisher;

  if (optional_location_msg.has_value)
  {
    // Define lookup-key to access publisher and ROS-topic
    std::string topic_name;
    if (params_.publisher_locations_to_individual_topics_per_sensor)
    {
      topic_name = params_.publisher_topic_locations; //+ "/" + received_rob2->device_position; lnl
    }
    else
    {
      topic_name = params_.publisher_topic_locations;
    }
    // lnl: test
    RCLCPP_INFO_STREAM(node_->get_logger(), "radar_locations_decoder_gen5_ros: creating publisher for topic: '" << topic_name << "'"); 

    // On-Demand create location publisher, if not yet exists
    // We do this on-demand since we do not (yet?) explicitely want to configure the exact list
    // of radars to be expected from the radar manager, we simply decode all sensor binaries
    // receifed on the yaml-specified topic
    if (map_radar_gen5_publisher_.find(topic_name) == map_radar_gen5_publisher_.end())
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "radar_locations_decoder_gen5_ros: creating publisher for topic: '" << topic_name << "'");      
      radar_gen5_publisher = node_->create_publisher<radar_gen5_msgs::msg::LocationInterface>(topic_name, params_.publisher_msg_queue_size);
      map_radar_gen5_publisher_[topic_name] = radar_gen5_publisher;          
    }
    else
    {
      radar_gen5_publisher = map_radar_gen5_publisher_[topic_name];
    }


    /*********lnl, timer*********/
    // radar_gen5_msgs::msg::Attribute buffer;
    // radar_gen5_msgs::msg::Attribute t_orig;
    // radar_gen5_msgs::msg::Attribute in_t;
    // radar_gen5_msgs::msg::Attribute out_t;
    // radar_gen5_msgs::msg::Attribute d_t;

    // radar_decoder_t_origin = optional_location_msg.value.radar_ecu_timestamp;

    // const auto time_temp_out = node_->now();
    // radar_decoder_out_t = time_temp_out;
    // radar_decoder_d_t = radar_decoder_out_t - radar_decoder_in_t;
    // t_orig.name = "radar_decoder_t_origin";
    // t_orig.nanoseconds = radar_decoder_t_origin*1.0e+09f;
    // in_t.name = "radar_decoder_in_t";
    // in_t.nanoseconds = radar_decoder_in_t.nanoseconds();

    // out_t.name = "radar_decoder_out_t";
    // out_t.nanoseconds = radar_decoder_out_t.nanoseconds();

    // d_t.name = "radar_decoder_d_t";
    // d_t.nanoseconds = radar_decoder_d_t.nanoseconds();
    
    // // const auto rob2_attributes_temp = received_rob2->attributes;
    // // for (auto attribute: received_rob2->attributes)
    // // {
    // //   buffer.name = attribute.name;
    // //   buffer.nanoseconds = attribute.nanoseconds;
    // //   optional_location_msg.value.attributes.push_back(buffer);
    // // }

    // optional_location_msg.value.attributes.push_back(t_orig);
    // optional_location_msg.value.attributes.push_back(in_t);
    // optional_location_msg.value.attributes.push_back(out_t);
    // optional_location_msg.value.attributes.push_back(d_t);
    /****************************/

    RCLCPP_DEBUG_STREAM(node_->get_logger(), "radar_locations_decoder_gen5_ros: publish location_interface_msg");
    radar_gen5_publisher->publish(optional_location_msg.value);
    diag_radar_decoder_locations_->tick(optional_location_msg.value.header.stamp);
  }
}

void RadarLocationsDecoderNode::run()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }
}
}
