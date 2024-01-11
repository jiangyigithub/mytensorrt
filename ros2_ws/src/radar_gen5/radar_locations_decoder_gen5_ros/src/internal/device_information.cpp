/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "radar_locations_decoder_gen5_ros/internal/device_information.hpp"

#include <iostream>

namespace radar_locations_decoder_gen5_ros
{
std::ostream& operator<<(std::ostream& out, const DeviceInformation& device)
{
  out << "sw_revision: " << device.software_version << ", device type: " << device.type
      << ", device position: " << device.position;
  return out;
}
}
