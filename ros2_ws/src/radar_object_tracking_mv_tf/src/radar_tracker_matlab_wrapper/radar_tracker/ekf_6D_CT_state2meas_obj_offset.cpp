//
// File: ekf_6D_CT_state2meas_obj_offset.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "ekf_6D_CT_state2meas_obj_offset.h"
#include "mgmt_init_track.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// EKF_6D_CT_STATE2MEAS conversion of state vector to measurement
//  same as ekf_6D_CA_state2meas, but with x-y offset in object coordinate
//  system
//
//  state vector = [px, py, heading, turn_rate, v_abs, 0].'
//  measurement = [range, alpha, v_radial].'
// Arguments    : const double state_vector[6]
//                double sens_offset_x
//                double sens_offset_y
//                double vx_ego
//                double x_offset_obj
//                double y_offset_obj
//                double obj_orntn
//                double measurement[3]
// Return Type  : void
//
void ekf_6D_CT_state2meas_obj_offset(const double state_vector[6], double
  sens_offset_x, double sens_offset_y, double vx_ego, double x_offset_obj,
  double y_offset_obj, double obj_orntn, double measurement[3])
{
  double py_sensor;
  double range;
  double px_sensor;

  //  single state vector
  //  augment state with offset
  py_sensor = std::sin(-obj_orntn);
  range = std::cos(-obj_orntn);

  //  in global coordinate system
  //  use standard function for measurement
  // EKF_6D_CT_STATE2MEAS conversion of state vector to measurement
  //  state vector = [px, py, heading, turn_rate, v_abs].'
  //  measurement = [range, alpha, v_radial].'
  px_sensor = (state_vector[0] + (x_offset_obj * range + y_offset_obj *
    py_sensor)) - sens_offset_x;

  //  x-position in sensor coordinate system
  py_sensor = (state_vector[1] + (-x_offset_obj * py_sensor + y_offset_obj *
    range)) - sens_offset_y;
  range = std::sqrt(px_sensor * px_sensor + py_sensor * py_sensor);

  //  vx=v_abs*cos(heading); vy=v_abs*sin(heading)
  //  x-velocity in sensor coordinate system
  //  y-velocity in sensor coordinate system
  //  v_r = v_vec.'p_vec/norm(p_vec)
  measurement[0] = range;
  measurement[1] = rt_atan2d_snf(py_sensor, px_sensor);
  measurement[2] = ((state_vector[4] * std::cos(state_vector[2]) - vx_ego) *
                    px_sensor + state_vector[4] * std::sin(state_vector[2]) *
                    py_sensor) / range;
}

//
// File trailer for ekf_6D_CT_state2meas_obj_offset.cpp
//
// [EOF]
//
