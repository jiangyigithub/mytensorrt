/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#pragma once

#include <cstdint>

namespace radar_gen5_common
{
struct ClockDefinition
{
  double ticks_per_second;
  // Tick count jumps to zero when tick_overflow is reached.
  std::uint64_t tick_overflow;
};

double convertOverflowToSeconds(const ClockDefinition& ticked_stamp);

namespace clocks
{
// Time is represented with 16 bits before and 16 bits after the comma. This means a second
// is divided into 2^16 ticks
// Overflow occurs after 2^32 ticks corresponding to 2^16 s = 18.2 h
constexpr ClockDefinition Base2Power16Clock{65536.0, 4294967296};

// Time is represented with 16 bits before and 16 bits after the comma. This means a second
// is divided into 2^16 ticks
// The overflow occurs when a MicrosecondsClock would overflow.
// Thus, the overflow point can be calculated according to the following formula:
// floor(2^32 microseconds * (10^-6 seconds/microseconds) * (2^16 ticks/second))
constexpr ClockDefinition Base2Power16ClockWithMicrosecondsOverflow{65536.0, 281474976U};

// Time is represented in milliseconds
// Overflow occurs after 2^32 milliseconds, corresponding to approximately 72 minutes
constexpr ClockDefinition MicrosecondsClock{1.0e6, 4294967296};

}  // namespace clocks

}  // namespace radar_gen5_common
