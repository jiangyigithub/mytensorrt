/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <cstdint>

#include "radar_gen5_common/clock_definition.hpp"

namespace radar_gen5_common
{
struct TickedTimestamp
{
  std::uint64_t tick_count;
  ClockDefinition clock;
};

double convertToSeconds(const TickedTimestamp& ticked_stamp);

TickedTimestamp unfoldOverflow(TickedTimestamp stamp, double expected_time_in_seconds);
}  // namespace radar_gen5_common
