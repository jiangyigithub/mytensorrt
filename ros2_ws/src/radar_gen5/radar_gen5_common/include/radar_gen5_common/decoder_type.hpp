/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <array>
#include <string>

namespace radar_gen5_common
{
enum class DecoderType
{
  LOCATIONS,
  MOUNTING_POSE,
  OBJECTS,
  ODOMETRY,
  ODOMETRY_HISTORY_BUFFER,
};

constexpr std::array<DecoderType, 5> getAllDecoderTypes()
{
  return {DecoderType::LOCATIONS,
          DecoderType::MOUNTING_POSE,
          DecoderType::OBJECTS,
          DecoderType::ODOMETRY,
          DecoderType::ODOMETRY_HISTORY_BUFFER};
}

std::string toString(DecoderType decoder_type);

DecoderType stringToDecoderType(const std::string& decoder_type);

}  // namespace radar_gen5_common
