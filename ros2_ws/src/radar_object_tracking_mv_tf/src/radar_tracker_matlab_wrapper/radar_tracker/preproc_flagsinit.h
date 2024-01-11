//
// File: preproc_flagsinit.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//
#ifndef PREPROC_FLAGSINIT_H
#define PREPROC_FLAGSINIT_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "radar_tracker_types.h"

// Function Declarations
extern void preproc_flagsinit(OBJECT_STRUCT *obj, double meas_list_t, double
  meas_list_dx_sens_offset, double meas_list_dy_sens_offset, double
  meas_list_ang_sens_offset, const double meas_list_fov_range[25], const double
  meas_list_fov_angle[25], double dt);

#endif

//
// File trailer for preproc_flagsinit.h
//
// [EOF]
//
