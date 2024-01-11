/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <radar_gen5_msgs/msg/location_interface.hpp>

namespace radar_locations_decoder_gen5_ros
{
struct OptionalLocationInterface
{
  bool has_value;
  radar_gen5_msgs::msg::LocationInterface value;
};

}  // namespace radar_locations_decoder_gen5_ros
