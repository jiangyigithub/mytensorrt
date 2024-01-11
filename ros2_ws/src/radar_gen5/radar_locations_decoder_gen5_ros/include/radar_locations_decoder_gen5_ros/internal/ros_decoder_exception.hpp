/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <stdexcept>
#include <string>

namespace radar_locations_decoder_gen5_ros
{
class RosDecoderException : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

}  // namespace radar_locations_decoder_gen5_ros
