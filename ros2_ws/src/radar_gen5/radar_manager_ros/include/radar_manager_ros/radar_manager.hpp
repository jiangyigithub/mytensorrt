/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef RADAR_MANAGER_HPP_
#define RADAR_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
// #include <xmlrpcpp/XmlRpcValue.h>

#include "radar_base.hpp"
#include "radar_manager_ros/manager_base.hpp"

namespace radar_manager
{
class CRadarManager : public CSensorMgrBase
{
  /* This Manager implements the SensorMgrBase interface.
   *
   */

public:
  static std::shared_ptr<CRadarManager> getInstance();

  virtual ~CRadarManager();

  virtual void Init();
  virtual void Start();
  virtual void Stop();

  void readConfiguration();
  void readDaqListFile(std::string daq_list_config_file, std::vector<CRadarBase::radar_daq_list> & daq_list_);

private:
  CRadarManager();

  static std::shared_ptr<CRadarManager> m_processor_singleton;

  std::vector<CRadarBase::radar_basic_info> device_info_;
};
}  // namespace radar_manager

#endif
