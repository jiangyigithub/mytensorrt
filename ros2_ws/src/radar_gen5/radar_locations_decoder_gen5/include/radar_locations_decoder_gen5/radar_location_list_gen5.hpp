/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <vector>

#include <vfc/core/vfc_types.hpp>
#include <vfc/core/vfc_float16_storage.hpp>

namespace radar_locations_decoder_gen5
{
   struct RadarLocationItemGen5
   {
   vfc::float32_t radial_distance = 0.0;
   vfc::float32_t radial_distance_var = 0.0;
   vfc::float32_t relative_radial_velocity = 0.0;
   vfc::float32_t relative_radial_velocity_var = 0.0;
   vfc::float32_t radial_distance_velocity_cov = 0.0;
   vfc::float32_t radial_distance_velocity_quality = 0.0;

   vfc::float32_t azimuth_angle = 0.0;
   vfc::float32_t azimuth_angle_deg = 0.0;
   vfc::float32_t azimuth_angle_var = 0.0;
   vfc::float32_t azimuth_angle_var_deg = 0.0;
   vfc::float32_t azimuth_angle_quality = 0.0;
   vfc::float32_t elevation_angle = 0.0;
   vfc::float32_t elevation_angle_deg = 0.0;
   vfc::float32_t elevation_angle_var = 0.0;
   vfc::float32_t elevation_angle_var_deg = 0.0;
   vfc::float32_t elevation_angle_quality = 0.0;

   vfc::float32_t rcs = 0.0;
   vfc::float32_t rssi = 0.0;

   vfc::float32_t radial_distance_spread = 0.0;
   vfc::float32_t relative_radial_velocity_spread = 0.0;
   vfc::float32_t distance_velocity_spread_orientation = 0.0;
 
   bool valid = false;
   bool multi_tar_azimuth_active = false;
   bool multi_tar_elevation_active = false;
   bool standing = false;
   vfc::uint8_t measurement_status = 0;
   };

   struct RadarLocationListGen5
   {
   vfc::uint16_t number_of_locations = 0;
   vfc::uint32_t abs_meas_time = 0;

   std::vector<RadarLocationItemGen5> item;

   };
} // namespace radar_locations_decoder_gen5

