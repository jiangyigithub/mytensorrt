//
// File: asso_quick_reject.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//
#ifndef ASSO_QUICK_REJECT_H
#define ASSO_QUICK_REJECT_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "radar_tracker_types.h"

// Function Declarations
extern void asso_quick_reject(double obj_pexist, const double obj_x[6], double
  obj_length, double obj_width, const double meas_list_potDoubleRefl[512], const
  double meas_list_meas[512], const double meas_list_dr[512], double
  meas_list_dx_sens_offset, double meas_list_dy_sens_offset, bool meas_reject
  [512]);

#endif

//
// File trailer for asso_quick_reject.h
//
// [EOF]
//
