/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <iosfwd>
#include <string>

namespace radar_locations_decoder_gen5_ros
{
struct DeviceInformation
{
  std::string software_version;
  std::string type;
  std::string position;
};

std::ostream& operator<<(std::ostream& out, const DeviceInformation& device);
}
