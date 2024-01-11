//
// File: ekf_6D_CA_state2meas_obj_offset.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//
#ifndef EKF_6D_CA_STATE2MEAS_OBJ_OFFSET_H
#define EKF_6D_CA_STATE2MEAS_OBJ_OFFSET_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "radar_tracker_types.h"

// Function Declarations
extern void ekf_6D_CA_state2meas_obj_offset(const double state_vector[6], double
  sens_offset_x, double sens_offset_y, double vx_ego, double x_offset_obj,
  double y_offset_obj, double obj_orntn, double measurement[3]);

#endif

//
// File trailer for ekf_6D_CA_state2meas_obj_offset.h
//
// [EOF]
//
