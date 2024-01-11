/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */


#include <fstream>
#include <algorithm>

// #include <ros/package.h>
// #include <xmlrpcpp/XmlRpcValue.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "radar_manager_ros/radar_manager.hpp"
#include <radar_msgs/msg/radar_factory_data_array.hpp>

using namespace radar_manager;

std::shared_ptr<CRadarManager> CRadarManager::m_processor_singleton;

std::shared_ptr<CRadarManager> CRadarManager::getInstance()
{
  if (CRadarManager::m_processor_singleton == NULL) {
    CRadarManager::m_processor_singleton.reset(new CRadarManager());
  }
  return CRadarManager::m_processor_singleton;
}

CRadarManager::CRadarManager()
{
  RCLCPP_DEBUG_STREAM(get_logger(), "[CRadarManager::CRadarManager()]");

  // In ROS2 parameters always have to be declared first (this does not mean they are set)
  declare_parameter("device_type");
  declare_parameter("device_position");
  declare_parameter("sw_revision");
  declare_parameter("ip_address");
  declare_parameter("daq_list_config");
  declare_parameter("port");

  readConfiguration();
}

CRadarManager::~CRadarManager()
{
  Stop();
}

void CRadarManager::readConfiguration()
{
  RCLCPP_DEBUG_STREAM(get_logger(), "[CRadarManager::readConfiguration()]");

  // read all devices configuration from parameter-yaml file:
  // XmlRpc::XmlRpcValue::iterator xml_it;
  // for (xml_it = sensor_params.begin(); xml_it != sensor_params.end(); ++xml_it) {
  CRadarBase::radar_basic_info device_info;

  // Parameter name and Value
  device_info.device_type = get_parameter("device_type").as_string();
  device_info.sw_revision = get_parameter("sw_revision").as_string();
  device_info.device_position = get_parameter("device_position").as_string();
  device_info.ip = get_parameter("ip_address").as_string();
  device_info.port = get_parameter("port").as_int();

  // Get daq_file_name and read it
  std::string path = ament_index_cpp::get_package_share_directory("radar_manager_ros") + "/params/";
  std::string daq_list_config_file = path + get_parameter("daq_list_config").as_string();
  readDaqListFile(daq_list_config_file, device_info.daq_list_);

  RCLCPP_DEBUG_STREAM(get_logger(), "[CRadarManager::readConfiguration()]"
                    << " device_type: " << device_info.device_type
                    << ", device_position: " << device_info.device_position
                    << ", sw_revision: " << device_info.sw_revision
                    << ", ip_adress: " << device_info.ip 
                    << ", port: " << device_info.port);

  device_info_.push_back(device_info);
  RCLCPP_DEBUG_STREAM(get_logger(), "[CRadarManager::readConfiguration()] Done");
}

void CRadarManager::readDaqListFile(std::string daq_list_config_file, std::vector<CRadarBase::radar_daq_list> & daq_list_)
{
  // read Device configuration from parameter-yaml file:
  std::ifstream file;
  std::string lineString;
  std::string odt_name;
  std::string param_file;
  int event_channel;
  file.open(daq_list_config_file);

  if (file.is_open()) {
    while (std::getline(file, lineString, '\n')) {
      std::stringstream ss;
      ss << lineString;

      if (lineString[0] != '#')  // ignore comments
      {
        CRadarBase::radar_daq_list daq_info;

        ss >> odt_name >> param_file >> std::hex >> event_channel;
        RCLCPP_DEBUG_STREAM(get_logger(), 
            "[CRadarManager::readDaqListFile] " << odt_name << ": event_channel:  " << std::hex
                                              << "0x" << event_channel
                                              << " param_file:  " << param_file);

        daq_info.event_channel = event_channel;
        daq_info.params_file = param_file;
        daq_list_.push_back(daq_info);
        RCLCPP_DEBUG_STREAM(get_logger(), 
            "[CRadarManager::readDaqListFile] current daq list has " << daq_list_.size()
                                              << " entries");
      }
    }
  } else {
    RCLCPP_ERROR_STREAM(get_logger(),
        "[CRadarManager::readDaqListFile] daq_list_config file not found: " 
                                              << daq_list_config_file);
  }
  file.close();
}

void CRadarManager::Init()
{
  RCLCPP_DEBUG_STREAM(get_logger(), "[CRadarManager::Init()]");

  if (device_info_.size() != 0) {
    for (std::vector<CRadarBase::radar_basic_info>::iterator device_it = device_info_.begin();
         device_it != device_info_.end();
         ++device_it) {           
      // add device
      if (device_it->device_type.compare("LRR5") == 0 ||
          device_it->device_type.compare("radar_gen5_plus") == 0) {

        std::cout << "[CRadarManager::Init()] Creating CRadarBase-Instance for device_type: " << device_it->device_type << std::endl;
        std::shared_ptr<CSensorBase> tmp = std::make_shared<CRadarBase>(*device_it, shared_from_this());
        sensor_list_.push_back(tmp);
      } else {
        RCLCPP_ERROR_STREAM(get_logger(),
             "[CRadarManager::Init()]: Initialization: Unknown deviceType: " << device_it->device_type);

        return;
      }
    }
    RCLCPP_DEBUG_STREAM(get_logger(), 
        "[CRadarManager::Init()]: " << (int)device_info_.size()
                                           << " radar devices successful initialized");
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), 
        "[CRadarManager::Init()]: No devices specified!");
  }
}

void CRadarManager::Start()
{
  // ros::Rate rate(10.0);  // measured in Hz
  rclcpp::WallRate rate(10.0);

  std::vector<CSensorBase*>::iterator sl_it;

  for (const auto& sensor : sensor_list_) {
    sensor->Start();
  }

  while (rclcpp::ok()) 
  {
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
}

void CRadarManager::Stop()
{
  for (const auto& sensor : sensor_list_) {
    sensor->Stop();
  }
}
