//
// File: asso_gating.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//
#ifndef ASSO_GATING_H
#define ASSO_GATING_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "radar_tracker_types.h"

// Function Declarations
extern void asso_gating(OBJECT_STRUCT *obj, const double y[3], const double
  sensor_pos_offset[2], double vx_ego, double y_aso[3], bool *is_associated,
  bool *mark_as_associated);

#endif

//
// File trailer for asso_gating.h
//
// [EOF]
//
