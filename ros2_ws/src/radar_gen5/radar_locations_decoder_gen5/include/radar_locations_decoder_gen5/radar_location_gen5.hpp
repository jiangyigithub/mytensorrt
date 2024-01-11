/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <vfc/core/vfc_types.hpp>
#include <vfc/core/vfc_float16_storage.hpp>

#include "radar_location_list_gen5.hpp"
#include "radar_location_sensing_state_gen5.hpp"

namespace radar_locations_decoder_gen5
{
   struct RadarLocationDataGen5
   {
      vfc::uint16_t reference_counter = 0;
      vfc::uint16_t sequence_number = 0;

      RadarLocationListGen5 location_list;
      RadarSensingStateGen5 sensing_state;
   };
} // namespace radar_locations_decoder_gen5

