/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <stdint.h>
#include "radar_location_gen5.hpp"
#include "radar_locations_decoder_gen5/decoder_exception.hpp"
#include "radar_locations_decoder_gen5/radar_location_sensing_state_gen5.hpp"


namespace radar_locations_decoder_gen5 {

class RadarLocationsDecoderImplInterface
{
public:
    RadarLocationsDecoderImplInterface() = default;
    virtual ~RadarLocationsDecoderImplInterface() = default;
    virtual RadarLocationDataGen5 decodeROBs(const uint8_t *data) const = 0;

    static const int RADAR_MEASUREMENT_OP_MODE = 20;

protected:

    template<typename DSP_LOCATION_DATA>
    void throwIfWrongOpMode(const DSP_LOCATION_DATA* buf_dsp_location_data_p) const
    {
        vfc::uint8_t op_mode = buf_dsp_location_data_p->m_SensState.getOpMode();
        if (op_mode != RADAR_MEASUREMENT_OP_MODE)
        {
            throw DecoderException("OpMode = " + std::to_string(op_mode) +
                                   "; OpMode have to be equal " + std::to_string(RADAR_MEASUREMENT_OP_MODE));
        }
    }

    template<typename DSP_LOCATION_DATA>
    void throwIfNoDataMeasured(const DSP_LOCATION_DATA* buf_dsp_location_data_p) const
    {
        if (!buf_dsp_location_data_p->m_SensState.DataMeasured)
        {
            throw DecoderException("No data is measured: m_SensState.DataMeasured = " +
                                   std::to_string(buf_dsp_location_data_p->m_SensState.DataMeasured));
        }
    }

    template<typename DSP_LOCATION_ITEM_TYPE>
    void convertCommonLocationItems(const DSP_LOCATION_ITEM_TYPE& dsp_loc_item, RadarLocationItemGen5& radar_location_item) const
    {
        radar_location_item.radial_distance = dsp_loc_item.getRadialDistance();
        radar_location_item.radial_distance_var = dsp_loc_item.getRadialDistanceVar();
        radar_location_item.relative_radial_velocity = dsp_loc_item.getRelativeRadialVelocity();
        radar_location_item.relative_radial_velocity_var = dsp_loc_item.getRelativeRadialVelocityVar();
        radar_location_item.radial_distance_velocity_cov = dsp_loc_item.getRadialDistanceVelocityCov();
        radar_location_item.radial_distance_velocity_quality = dsp_loc_item.getRadialDistanceVelocityQuality();

        radar_location_item.azimuth_angle = dsp_loc_item.getAzimuthAngle();
        radar_location_item.azimuth_angle_deg = dsp_loc_item.getAzimuthAngleDeg();
        radar_location_item.azimuth_angle_var = dsp_loc_item.getAzimuthAngleVar();
        radar_location_item.azimuth_angle_var_deg = dsp_loc_item.getAzimuthAngleVarDeg();
        radar_location_item.azimuth_angle_quality = dsp_loc_item.getAzimuthAngleQuality();
        radar_location_item.elevation_angle = dsp_loc_item.getElevationAngle();
        radar_location_item.elevation_angle_deg = dsp_loc_item.getElevationAngleDeg();
        radar_location_item.elevation_angle_var = dsp_loc_item.getElevationAngleVar();
        radar_location_item.elevation_angle_var_deg = dsp_loc_item.getElevationAngleVarDeg();
        radar_location_item.elevation_angle_quality = dsp_loc_item.getElevationAngleQuality();

        radar_location_item.rcs = dsp_loc_item.getRcs();
        radar_location_item.rssi = dsp_loc_item.getRssi();

        radar_location_item.valid = dsp_loc_item.isValid();
        radar_location_item.multi_tar_azimuth_active = dsp_loc_item.isMultiTargetAzimuthActive();
        radar_location_item.multi_tar_elevation_active = dsp_loc_item.isMultiTargetElevationActive();
        radar_location_item.standing = dsp_loc_item.isStanding();
        radar_location_item.measurement_status = dsp_loc_item.measStatus;
    }

    template<typename SENSING_STATE_DECODED>
    void fillSensingStateMessage(const SENSING_STATE_DECODED & sensingStateDecoder, RadarSensingStateGen5 &sensingStateMessageOut ) const
    {
        // fill status
        sensingStateMessageOut.op_mode = sensingStateDecoder.getOpMode();
        sensingStateMessageOut.data_measured = sensingStateDecoder.DataMeasured;
        sensingStateMessageOut.fac_thermal_degradation = vfc::float32_t( sensingStateDecoder.getfacThermalDeg() );

        // fill field_of_view
        for( unsigned i=0; i<GEN5_AZIMUTH_FOV_SIZE; i++ )
          sensingStateMessageOut.field_of_view.maximum_fov_range_azimuth[i] = vfc::float32_t( sensingStateDecoder.FieldOfView_st.dMaxThetaView[i] );
        for( unsigned i=0; i<GEN5_AZIMUTH_FOV_SIZE; i++ )
          sensingStateMessageOut.field_of_view.fov_azimuth_angles[i] = vfc::float32_t( sensingStateDecoder.FieldOfView_st.thetaViewAry[i] );
        for( unsigned i=0; i<GEN5_ELEVATION_FOV_SIZE; i++ )
          sensingStateMessageOut.field_of_view.range_scaling_elevation[i] = vfc::float32_t( sensingStateDecoder.FieldOfView_st.facDMaxPhiView[i] );
        for( unsigned i=0; i<GEN5_ELEVATION_FOV_SIZE; i++ )
          sensingStateMessageOut.field_of_view.fov_elevation_angles[i] = vfc::float32_t( sensingStateDecoder.FieldOfView_st.phiViewAry[i] );

        // fill interference_status
        sensingStateMessageOut.interference_status.indicator_status        = sensingStateDecoder.Interference_st.IntfrIndcrStatus;
        sensingStateMessageOut.interference_status.field_of_view_reduction = vfc::float32_t( sensingStateDecoder.Interference_st.IntfrIndcr );

        // fill misalignment
        sensingStateMessageOut.misalignment.estimation_status                     = sensingStateDecoder.Misalignment_st.malStatus;
        sensingStateMessageOut.misalignment.azimuth_angle_misalignment            = vfc::float32_t( sensingStateDecoder.Misalignment_st.thetaMalAng );
        sensingStateMessageOut.misalignment.azimuth_angle_misalignment_variance   = vfc::float32_t( sensingStateDecoder.Misalignment_st.thetaMalAngVar );
        sensingStateMessageOut.misalignment.elevation_angle_misalignment          = vfc::float32_t( sensingStateDecoder.Misalignment_st.phiMalAng );
        sensingStateMessageOut.misalignment.elevation_angle_misalignment_variance = vfc::float32_t( sensingStateDecoder.Misalignment_st.phiMalAngVar );

        // fill modulation_performance
        sensingStateMessageOut.modulation_performance.active_detection_measurement_program_id = sensingStateDecoder.ModulationPerformance_st.DmpID;
        sensingStateMessageOut.modulation_performance.active_modulation_id                    = sensingStateDecoder.ModulationPerformance_st.ModID;
        sensingStateMessageOut.modulation_performance.fov_distance_range_scaling_factor       = vfc::float32_t( sensingStateDecoder.ModulationPerformance_st.facDMaxModDur );
        sensingStateMessageOut.modulation_performance.distance_precision                      = vfc::float32_t( sensingStateDecoder.ModulationPerformance_st.dPrec );
        sensingStateMessageOut.modulation_performance.distance_seperability                   = vfc::float32_t( sensingStateDecoder.ModulationPerformance_st.dSprblty );
        sensingStateMessageOut.modulation_performance.distance_minimum_value                  = vfc::float32_t( sensingStateDecoder.ModulationPerformance_st.dmin );
        sensingStateMessageOut.modulation_performance.distance_maximum_value                  = vfc::float32_t( sensingStateDecoder.ModulationPerformance_st.dmax );
        sensingStateMessageOut.modulation_performance.velocity_precision                      = vfc::float32_t( sensingStateDecoder.ModulationPerformance_st.vPrec );
        sensingStateMessageOut.modulation_performance.velocity_seperability                   = vfc::float32_t( sensingStateDecoder.ModulationPerformance_st.vSprblty );
        sensingStateMessageOut.modulation_performance.velocity_minimum_value                  = vfc::float32_t( sensingStateDecoder.ModulationPerformance_st.vmin );
        sensingStateMessageOut.modulation_performance.velocity_maximum_value                  = vfc::float32_t( sensingStateDecoder.ModulationPerformance_st.vmax );
        sensingStateMessageOut.modulation_performance.distance_velocity_covariance            = vfc::float32_t( sensingStateDecoder.ModulationPerformance_st.dvCov );
    }

};

} // namespace radar_locations_decoder_gen5
