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


namespace radar_locations_decoder_gen5 {

class DecoderException : public std::runtime_error
{
public:
  DecoderException(const std::string& error_message) : std::runtime_error(error_message){}
};

class UnknownSoftwareVersionException : public DecoderException
{
public:
  UnknownSoftwareVersionException(const std::string& error_message) : DecoderException(error_message){}
};

} // namespace radar_locations_decoder_gen5
