/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "radar_gen5_common/ticked_timestamp.hpp"

#include <limits>

namespace radar_gen5_common
{
namespace
{
constexpr auto MAXIMUM_REPRESENTABLE_OVERFLOW =
    std::numeric_limits<decltype(TickedTimestamp::clock.tick_overflow)>::max();

bool hasAlreadyBeenUnfolded(const TickedTimestamp& stamp)
{
  return stamp.clock.tick_overflow == MAXIMUM_REPRESENTABLE_OVERFLOW;
}
}  // namespace

double convertToSeconds(const TickedTimestamp& ticked_stamp)
{
  return static_cast<double>(ticked_stamp.tick_count) / ticked_stamp.clock.ticks_per_second;
}

TickedTimestamp unfoldOverflow(TickedTimestamp stamp, double expected_time_in_seconds)
{
  // Timestamps that have already been unfolded can't be unfolded again.
  if (hasAlreadyBeenUnfolded(stamp)) {
    return stamp;
  }
  const double overflow_in_seconds = convertOverflowToSeconds(stamp.clock);
  const double maximum_difference_to_expected = 0.5 * overflow_in_seconds;
  while (expected_time_in_seconds - convertToSeconds(stamp) > maximum_difference_to_expected) {
    stamp.tick_count += stamp.clock.tick_overflow;
  }
  stamp.clock.tick_overflow = MAXIMUM_REPRESENTABLE_OVERFLOW;
  return stamp;
}

}  // namespace radar_gen5_common
