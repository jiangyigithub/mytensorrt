/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef MANAGER_BASE_HPP_
#define MANAGER_BASE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "radar_manager_ros/sensor_base.hpp"

class CSensorMgrBase : public rclcpp::Node
{
  /* This is the SensorMgrBase class.
   * All individual sensor managers should implement/derive this class
   */

public:
  CSensorMgrBase() : rclcpp::Node("radar_manager_ros"), sensor_list_(){};
  
  virtual ~CSensorMgrBase()
  {
    sensor_list_.clear();
  };

  virtual void Init()
  {
    std::vector<std::shared_ptr<CSensorBase>>::iterator sl_it;

    for (sl_it = sensor_list_.begin(); sl_it != sensor_list_.end(); ++sl_it) {
      (*sl_it)->Init();
    }
  }
  virtual void Start()
  {
    std::vector<std::shared_ptr<CSensorBase>>::iterator sl_it;

    for (sl_it = sensor_list_.begin(); sl_it != sensor_list_.end(); ++sl_it) {
      (*sl_it)->Start();
    }
  }
  virtual void Stop()
  {
    std::vector<std::shared_ptr<CSensorBase>>::iterator sl_it;

    for (sl_it = sensor_list_.begin(); sl_it != sensor_list_.end(); ++sl_it) {
      (*sl_it)->Stop();
    }
  }

  std::vector<std::shared_ptr<CSensorBase>> sensor_list_;
};

#endif
