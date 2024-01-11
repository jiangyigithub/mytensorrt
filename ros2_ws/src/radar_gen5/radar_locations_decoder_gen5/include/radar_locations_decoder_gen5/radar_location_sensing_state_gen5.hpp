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

namespace radar_locations_decoder_gen5
{
    static const int GEN5_AZIMUTH_FOV_SIZE = 25;
    static const int GEN5_ELEVATION_FOV_SIZE = 11;

    struct FieldOfView {
      vfc::float32_t maximum_fov_range_azimuth [GEN5_AZIMUTH_FOV_SIZE] = {};
      vfc::float32_t fov_azimuth_angles [GEN5_AZIMUTH_FOV_SIZE] = {};
      vfc::float32_t range_scaling_elevation [GEN5_ELEVATION_FOV_SIZE] = {};
      vfc::float32_t fov_elevation_angles [GEN5_ELEVATION_FOV_SIZE] = {};
    };
    struct BlindnessMessage {
      vfc::uint16_t num_bins_detection_threshold = 0;
      bool num_bins_detection_threshold_valid = false;
      vfc::uint16_t rcs_moving_object_model_deviation_indicator = 0;
      bool rcs_moving_object_model_deviation_indicator_valid = false;
      vfc::uint16_t azimuth_moving_object_model_deviation = 0;
      bool azimuth_moving_object_model_deviation_valid = false;
      vfc::uint16_t azimuth_static_object_model_deviation = 0;
      bool azimuth_static_object_model_deviation_valid = false;
    };
    struct InterferenceStatus {
      vfc::uint8_t indicator_status = 0;
      vfc::float32_t field_of_view_reduction = 0.0;
    };
    struct Misalignment {
      vfc::uint8_t estimation_status = 0;
      vfc::float32_t azimuth_angle_misalignment = 0.0;
      vfc::float32_t azimuth_angle_misalignment_variance = 0.0;
      vfc::float32_t elevation_angle_misalignment = 0.0;
      vfc::float32_t elevation_angle_misalignment_variance = 0.0;
    };
    struct ModulationPerformance {
      vfc::uint8_t active_detection_measurement_program_id = 0;
      vfc::uint16_t active_modulation_id = 0;
      vfc::float32_t fov_distance_range_scaling_factor = 0.0;
      vfc::float32_t distance_precision = 0.0;
      vfc::float32_t distance_seperability = 0.0;
      vfc::float32_t distance_minimum_value = 0.0;
      vfc::float32_t distance_maximum_value = 0.0;
      vfc::float32_t velocity_precision = 0.0;
      vfc::float32_t velocity_seperability = 0.0;
      vfc::float32_t velocity_minimum_value = 0.0;
      vfc::float32_t velocity_maximum_value = 0.0;
      vfc::float32_t distance_velocity_covariance = 0.0;
    };

   struct RadarSensingStateGen5
   {
    vfc::uint8_t op_mode = 0;
    bool data_measured = false;
    vfc::float16_storage_t fac_thermal_degradation = 0.0;
    //vfc::float32_t view_distance; // is a function!

    FieldOfView field_of_view;
    BlindnessMessage blindness_message;
    InterferenceStatus interference_status;
    Misalignment misalignment;
    ModulationPerformance modulation_performance;
   };
} // namespace radar_locations_decoder_gen5

