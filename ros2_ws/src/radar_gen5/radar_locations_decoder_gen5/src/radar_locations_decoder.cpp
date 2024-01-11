/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "radar_locations_decoder_gen5/radar_locations_decoder.hpp"

#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X391.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X310_RC1_local.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X230.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X220.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X173.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X171_DV1_local.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X170.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X169.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X168_local.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_B167_local.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X167.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_Y163.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_D161.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X123.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_X151.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_Y140.hpp"
#include "radar_locations_decoder_gen5/radar_locations_decoder_impl_RC18041.hpp"

namespace radar_locations_decoder_gen5 {

RadarDecoder::RadarDecoder()
{
    decoders_map_.insert(std::make_pair("RC18041", std::make_unique<RadarLocationsDecoderImplRC18041>()));
    decoders_map_.insert(std::make_pair("RC18091", std::make_unique<RadarLocationsDecoderImplD161>()));           // same format as VWMQB37W D161
    decoders_map_.insert(std::make_pair("BJEV_N60_R6_RC01", std::make_unique<RadarLocationsDecoderImplX391>()));  // same format as VWMQB37W X391
    decoders_map_.insert(std::make_pair("BJEV_N60_BL09_V2", std::make_unique<RadarLocationsDecoderImplX391>()));  // same format as VWMQB37W X391
    // VWMQB37W
    decoders_map_.insert(std::make_pair("X395", std::make_unique<RadarLocationsDecoderImplX391>())); // no change compared to X391
    decoders_map_.insert(std::make_pair("Y394_RDC2", std::make_unique<RadarLocationsDecoderImplX391>())); // no change compared to X391
    decoders_map_.insert(std::make_pair("X391", std::make_unique<RadarLocationsDecoderImplX391>()));
    decoders_map_.insert(std::make_pair("X310_RC1_local", std::make_unique<RadarLocationsDecoderImplX310RC1Local>()));
    decoders_map_.insert(std::make_pair("X230", std::make_unique<RadarLocationsDecoderImplX230>()));
    decoders_map_.insert(std::make_pair("X220", std::make_unique<RadarLocationsDecoderImplX220>()));
    decoders_map_.insert(std::make_pair("X173", std::make_unique<RadarLocationsDecoderImplX173>()));
    decoders_map_.insert(std::make_pair("X171_DV1_local", std::make_unique<RadarLocationsDecoderImplX171DV1Local>()));
    decoders_map_.insert(std::make_pair("X170", std::make_unique<RadarLocationsDecoderImplX170>()));
    decoders_map_.insert(std::make_pair("X169", std::make_unique<RadarLocationsDecoderImplX169>()));
    decoders_map_.insert(std::make_pair("X168_local", std::make_unique<RadarLocationsDecoderImplX168Local>()));
    decoders_map_.insert(std::make_pair("B167_local", std::make_unique<RadarLocationsDecoderImplB167Local>()));
    decoders_map_.insert(std::make_pair("X167", std::make_unique<RadarLocationsDecoderImplX167>()));
    decoders_map_.insert(std::make_pair("Y163", std::make_unique<RadarLocationsDecoderImplY163>()));
    decoders_map_.insert(std::make_pair("D161", std::make_unique<RadarLocationsDecoderImplD161>()));
    decoders_map_.insert(std::make_pair("X151", std::make_unique<RadarLocationsDecoderImplX151>()));
    decoders_map_.insert(std::make_pair("Y140", std::make_unique<RadarLocationsDecoderImplY140>()));
    decoders_map_.insert(std::make_pair("X123", std::make_unique<RadarLocationsDecoderImplX123>()));
}

RadarLocationDataGen5 RadarDecoder::decodeROBs(const std::string sw_version, const uint8_t *data)
{
    if (decoders_map_.find(sw_version) == decoders_map_.end())
    {
        throw UnknownSoftwareVersionException("Decoder for SW-Version " + sw_version + " does not exist");
    }

    return decoders_map_[sw_version]->decodeROBs(data);
}

} // namespace radar_locations_decoder_gen5
