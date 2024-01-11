//
// File: radar_tracker_data.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "radar_tracker_data.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"

// Variable Definitions
unsigned int state[625];
const e_struct_T r = { 0.0,            // loc_nr
  { 0.0, 0.0, 0.0 },                   // y
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },// R
  0.0,                                 // PDH1
  0.0,                                 // PDH0
  0.0,                                 // potGhost
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },// S
  { 0.0, 0.0, 0.0 }                    // d2
};

bool isInitialized_radar_tracker = false;

//
// File trailer for radar_tracker_data.cpp
//
// [EOF]
//
