/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "radar_location_gen5.hpp"
#include "radar_locations_decoder_impl_interface.hpp"

namespace radar_locations_decoder_gen5
{
class RadarDecoder
{
public:
  // normalization factor which was used in radar ecu (Norm_tAbsUL_ul):
  // time in secs was multiplied by this factor
  static constexpr float NORMALIZATION_FACTOR_ECU_TIMESTAMP = 65536.0f;

  RadarDecoder();
  RadarLocationDataGen5 decodeROBs(const std::string sw_version, const uint8_t* data);

private:
  std::unordered_map<std::string, std::unique_ptr<RadarLocationsDecoderImplInterface>>
      decoders_map_;
};

}  // namespace radar_locations_decoder_gen5
