/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include <radar_locations_decoder_gen5/radar_locations_decoder_impl_X310_RC1_local.hpp>
#include <radar_locations_decoder_gen5/internal/radar_gen5_X310_RC1_local_definitions/dsp_LocationInterface_X310_RC1_local.hpp>

namespace radar_locations_decoder_gen5 {


RadarLocationDataGen5 RadarLocationsDecoderImplX310RC1Local::decodeROBs(const uint8_t *data) const
{
    const X310_RC1_local::Dsp::LocationInterface *buf_dsp_location_data_p = (const X310_RC1_local::Dsp::LocationInterface *)data;

    throwIfWrongOpMode(buf_dsp_location_data_p);
    throwIfNoDataMeasured(buf_dsp_location_data_p);

    RadarLocationDataGen5 radar_locations_gen5;

    radar_locations_gen5.reference_counter = buf_dsp_location_data_p->m_referenceCounter;
    radar_locations_gen5.sequence_number = buf_dsp_location_data_p->m_sequenceNumber;

    const auto& location_list = buf_dsp_location_data_p->m_LocationList;

    radar_locations_gen5.location_list.number_of_locations = location_list.getNumberOfLocations();
    radar_locations_gen5.location_list.abs_meas_time = location_list.getAbsMeasTime();

    for (size_t locIdx = 0; locIdx < location_list.getNumberOfLocations(); locIdx++)
    {
        RadarLocationItemGen5 item_curr;
        const auto& dsp_loc_item = location_list.Item[locIdx];

        convertCommonLocationItems(dsp_loc_item, item_curr);

        // Convert not common members
        // adjust this for each new SW-Version
        item_curr.radial_distance_spread = 0.0;
        item_curr.relative_radial_velocity_spread = 0.0;
        item_curr.distance_velocity_spread_orientation = 0.0;

        radar_locations_gen5.location_list.item.push_back(item_curr);
    }

    RadarLocationsDecoderImplInterface::fillSensingStateMessage(buf_dsp_location_data_p->m_SensState, radar_locations_gen5.sensing_state );

    return radar_locations_gen5;
}
} // namespace radar_locations_decoder_gen5
