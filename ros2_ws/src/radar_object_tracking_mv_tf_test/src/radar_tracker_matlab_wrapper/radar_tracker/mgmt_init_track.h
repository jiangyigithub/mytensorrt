//
// File: mgmt_init_track.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//
#ifndef MGMT_INIT_TRACK_H
#define MGMT_INIT_TRACK_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "radar_tracker_types.h"

// Function Declarations
extern void mgmt_init_track(OBJECT_STRUCT obj[50], const double
  measurements_asso[512], const double measurements_potDoubleRefl[512], const
  double measurements_meas[512], const double measurements_dr[512], const double
  measurements_vr[512], const double measurements_dBRcs[512], double
  measurements_t, double measurements_vx_ego, double measurements_dx_sens_offset,
  double measurements_dy_sens_offset, const double measurements_alpha[512],
  double dt);
extern void trackletManagementInit_init();

#endif

//
// File trailer for mgmt_init_track.h
//
// [EOF]
//
