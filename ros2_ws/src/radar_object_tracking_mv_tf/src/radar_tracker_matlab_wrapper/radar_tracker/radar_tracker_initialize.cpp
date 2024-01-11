//
// File: radar_tracker_initialize.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "radar_tracker_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "mgmt_init_track.h"
#include "radar_tracker.h"
#include "radar_tracker_data.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void radar_tracker_initialize()
{
  rt_InitInfAndNaN();
  c_eml_rand_mt19937ar_stateful_i();
  trackletManagementInit_init();
  isInitialized_radar_tracker = true;
}

//
// File trailer for radar_tracker_initialize.cpp
//
// [EOF]
//
