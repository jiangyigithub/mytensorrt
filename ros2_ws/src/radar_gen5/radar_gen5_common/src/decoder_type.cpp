/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "radar_gen5_common/decoder_type.hpp"

#include <stdexcept>

namespace radar_gen5_common
{
namespace strings
{
const std::string LOCATION_DECODER = "location_decoder";
const std::string MOUNTING_POSE_DECODER = "mounting_pose_decoder";
const std::string OBJECTS_DECODER = "objects_decoder";
const std::string ODOMETRY_DECODER = "odometry_decoder";
const std::string ODOMETRY_HISTORY_BUFFER_DECODER = "odometry_history_buffer_decoder";
}  // namespace strings

std::string toString(DecoderType decoder_type)
{
  switch (decoder_type) {
    case DecoderType::LOCATIONS:
      return strings::LOCATION_DECODER;
    case DecoderType::MOUNTING_POSE:
      return strings::MOUNTING_POSE_DECODER;
    case DecoderType::OBJECTS:
      return strings::OBJECTS_DECODER;
    case DecoderType::ODOMETRY:
      return strings::ODOMETRY_DECODER;
    case DecoderType::ODOMETRY_HISTORY_BUFFER:
      return strings::ODOMETRY_HISTORY_BUFFER_DECODER;
    default:
      throw std::out_of_range("unknown decoder type enum value");
  }
}

DecoderType stringToDecoderType(const std::string& decoder_type)
{
  if (decoder_type == strings::LOCATION_DECODER) {
    return DecoderType::LOCATIONS;
  }
  if (decoder_type == strings::MOUNTING_POSE_DECODER) {
    return DecoderType::MOUNTING_POSE;
  }
  if (decoder_type == strings::OBJECTS_DECODER) {
    return DecoderType::MOUNTING_POSE;
  }
  if (decoder_type == strings::ODOMETRY_DECODER) {
    return DecoderType::ODOMETRY;
  }
  if (decoder_type == strings::ODOMETRY_HISTORY_BUFFER_DECODER) {
    return DecoderType::ODOMETRY_HISTORY_BUFFER;
  }
  throw std::out_of_range("unknown decoder type: " + decoder_type);
}

}  // namespace radar_gen5_common
