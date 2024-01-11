//
// File: eml_rand_mt19937ar_stateful.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "eml_rand_mt19937ar_stateful.h"
#include "radar_tracker.h"
#include "radar_tracker_data.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void c_eml_rand_mt19937ar_stateful_i()
{
  unsigned int b_r;
  std::memset(&state[0], 0, 625U * sizeof(unsigned int));
  b_r = 5489U;
  state[0] = 5489U;
  for (int mti = 0; mti < 623; mti++) {
    b_r = ((b_r ^ b_r >> 30U) * 1812433253U + mti) + 1U;
    state[mti + 1] = b_r;
  }

  state[624] = 624U;
}

//
// File trailer for eml_rand_mt19937ar_stateful.cpp
//
// [EOF]
//
