/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include <memory>

#include <radar_manager_ros/radar_manager.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace radar_manager;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); //, "radar_manager"

  std::shared_ptr<CRadarManager> radar_manager;
  try
  {
    radar_manager = CRadarManager::getInstance();

    RCLCPP_DEBUG(radar_manager->get_logger(), "radar_manager->Init()");
    radar_manager->Init();

    RCLCPP_DEBUG(radar_manager->get_logger(), "radar_manager->Start()");
    radar_manager->Start();
  }
  catch (const std::runtime_error &e)
  {
    RCLCPP_ERROR(radar_manager->get_logger(), "Terminating %s: %s", argv[0], e.what());
  }

  if (radar_manager)
  {
    radar_manager->Stop();
    radar_manager.reset();
  }
}
