//
// File: asso_gating.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "asso_gating.h"
#include "mgmt_init_track.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// Gating with fixed window-size
//  y = [dr,alpha,vr]'
//  x = [dx vx ax dy vy ay]'
// Arguments    : OBJECT_STRUCT *obj
//                const double y[3]
//                const double sensor_pos_offset[2]
//                double vx_ego
//                double y_aso[3]
//                bool *is_associated
//                bool *mark_as_associated
// Return Type  : void
//
void asso_gating(OBJECT_STRUCT *obj, const double y[3], const double
                 sensor_pos_offset[2], double vx_ego, double y_aso[3], bool
                 *is_associated, bool *mark_as_associated)
{
  double obj_angle;
  double range_gate_abs;
  double angle_gate_abs;
  double range;
  double v_radial_max;
  double vr_absGate;
  double angle_max;
  double py_sensor;
  double range_max;
  double b_py_sensor;
  double px_sensor;
  double range_min;
  double c_py_sensor;
  double b_range;
  double c_range;
  double measurement1_idx_2;
  double b_px_sensor;
  double c_px_sensor;
  double py_sensor_tmp;
  double d_py_sensor;
  double d_range;
  double measurement2_idx_2;
  double d_px_sensor;
  double measurement3_idx_2;
  double state_augmented_idx_1;
  double state_augmented_idx_4;
  double corner_ranges[4];
  bool b;
  int idx;
  int k;
  bool exitg1;
  y_aso[0] = rtNaN;
  y_aso[1] = rtNaN;
  y_aso[2] = rtNaN;
  *is_associated = false;
  *mark_as_associated = false;
  obj_angle = rt_atan2d_snf(obj->x[3], obj->x[0]);

  //  only rough angle estimate to avoid 2pi jump, not including sensor position offset 
  range_gate_abs = y[0] / 20.0;
  if (!(range_gate_abs > 4.0)) {
    range_gate_abs = 4.0;
  }

  angle_gate_abs = 0.034906585039886591;

  //  2 degree
  range = std::sqrt(obj->x[0] * obj->x[0] + obj->x[3] * obj->x[3]) / 10.0;
  v_radial_max = std::abs(y[2] * 0.2);
  if ((range > v_radial_max) || rtIsNaN(v_radial_max)) {
    vr_absGate = range;
  } else {
    vr_absGate = v_radial_max;
  }

  if (!(vr_absGate > 2.0)) {
    vr_absGate = 2.0;
  }

  if (y[0] < 15.0) {
    //  ignoring elevation makes a difference for near objects
    vr_absGate = (vr_absGate + 0.5 * std::abs(vx_ego)) + 0.2 * std::abs(y[2]);
    angle_gate_abs = (17.0 - y[0]) * 3.1415926535897931 / 180.0;

    //  2 degree at r=15, 15 degree at r=2
  }

  //  worst case gating:
  //  check four corner points of object
  v_radial_max = obj->length / 2.0;
  range = obj->width / 2.0;

  // EKF_6D_CA_STATE2MEAS_OBJ_OFFSET conversion of state vector to measurement
  //  same as ekf_6D_CA_state2meas, but with x-y offset in object coordinate
  //  system
  //
  //  state vector = [px, vx, ax, py, vy, ay].'
  //  measurement = [range, alpha, v_radial].'
  //  single state vector
  //  augment state with offset
  angle_max = std::sin(-obj->psi);
  py_sensor = std::cos(-obj->psi);

  //  in global coordinate system
  //  use standard function for measurement
  // EKF_6D_CA_STATE2MEAS conversion of state vector to measurement
  //  state vector = [px, vx, ax, py, vy, ay].'
  //  measurement = [range, alpha, v_radial].'
  range_max = v_radial_max * py_sensor;
  b_py_sensor = range * angle_max;
  px_sensor = (obj->x[0] + (range_max + b_py_sensor)) - sensor_pos_offset[0];

  //  x-position in sensor coordinate system
  v_radial_max = -v_radial_max * angle_max;
  range_min = range * py_sensor;
  c_py_sensor = (obj->x[3] + (v_radial_max + range_min)) - sensor_pos_offset[1];
  b_range = std::sqrt(px_sensor * px_sensor + c_py_sensor * c_py_sensor);

  //  x-velocity in sensor coordinate system
  //  y-velocity in sensor coordinate system
  //  v_r = v_vec.'p_vec/norm(p_vec)
  c_range = obj->x[1] - vx_ego;
  measurement1_idx_2 = (c_range * px_sensor + obj->x[4] * c_py_sensor) / b_range;
  range = -obj->width / 2.0;

  // EKF_6D_CA_STATE2MEAS_OBJ_OFFSET conversion of state vector to measurement
  //  same as ekf_6D_CA_state2meas, but with x-y offset in object coordinate
  //  system
  //
  //  state vector = [px, vx, ax, py, vy, ay].'
  //  measurement = [range, alpha, v_radial].'
  //  single state vector
  //  augment state with offset
  //  in global coordinate system
  //  use standard function for measurement
  // EKF_6D_CA_STATE2MEAS conversion of state vector to measurement
  //  state vector = [px, vx, ax, py, vy, ay].'
  //  measurement = [range, alpha, v_radial].'
  b_px_sensor = range * angle_max;
  c_px_sensor = (obj->x[0] + (range_max + b_px_sensor)) - sensor_pos_offset[0];

  //  x-position in sensor coordinate system
  py_sensor_tmp = range * py_sensor;
  d_py_sensor = (obj->x[3] + (v_radial_max + py_sensor_tmp)) -
    sensor_pos_offset[1];
  d_range = std::sqrt(c_px_sensor * c_px_sensor + d_py_sensor * d_py_sensor);

  //  x-velocity in sensor coordinate system
  //  y-velocity in sensor coordinate system
  //  v_r = v_vec.'p_vec/norm(p_vec)
  measurement2_idx_2 = (c_range * c_px_sensor + obj->x[4] * d_py_sensor) /
    d_range;
  v_radial_max = -obj->length / 2.0;

  // EKF_6D_CA_STATE2MEAS_OBJ_OFFSET conversion of state vector to measurement
  //  same as ekf_6D_CA_state2meas, but with x-y offset in object coordinate
  //  system
  //
  //  state vector = [px, vx, ax, py, vy, ay].'
  //  measurement = [range, alpha, v_radial].'
  //  single state vector
  //  augment state with offset
  //  in global coordinate system
  //  use standard function for measurement
  // EKF_6D_CA_STATE2MEAS conversion of state vector to measurement
  //  state vector = [px, vx, ax, py, vy, ay].'
  //  measurement = [range, alpha, v_radial].'
  range_max = v_radial_max * py_sensor;
  d_px_sensor = (obj->x[0] + (range_max + b_py_sensor)) - sensor_pos_offset[0];

  //  x-position in sensor coordinate system
  v_radial_max = -v_radial_max * angle_max;
  b_py_sensor = (obj->x[3] + (v_radial_max + range_min)) - sensor_pos_offset[1];
  range = std::sqrt(d_px_sensor * d_px_sensor + b_py_sensor * b_py_sensor);

  //  x-velocity in sensor coordinate system
  //  y-velocity in sensor coordinate system
  //  v_r = v_vec.'p_vec/norm(p_vec)
  measurement3_idx_2 = (c_range * d_px_sensor + obj->x[4] * b_py_sensor) / range;

  // EKF_6D_CA_STATE2MEAS_OBJ_OFFSET conversion of state vector to measurement
  //  same as ekf_6D_CA_state2meas, but with x-y offset in object coordinate
  //  system
  //
  //  state vector = [px, vx, ax, py, vy, ay].'
  //  measurement = [range, alpha, v_radial].'
  //  single state vector
  //  augment state with offset
  //  in global coordinate system
  state_augmented_idx_1 = obj->x[1];
  state_augmented_idx_4 = obj->x[4];

  //  use standard function for measurement
  // EKF_6D_CA_STATE2MEAS conversion of state vector to measurement
  //  state vector = [px, vx, ax, py, vy, ay].'
  //  measurement = [range, alpha, v_radial].'
  b_px_sensor = (obj->x[0] + (range_max + b_px_sensor)) - sensor_pos_offset[0];

  //  x-position in sensor coordinate system
  py_sensor = (obj->x[3] + (v_radial_max + py_sensor_tmp)) - sensor_pos_offset[1];
  c_range = std::sqrt(b_px_sensor * b_px_sensor + py_sensor * py_sensor);

  //  x-velocity in sensor coordinate system
  //  y-velocity in sensor coordinate system
  //  v_r = v_vec.'p_vec/norm(p_vec)
  //  range interval
  corner_ranges[0] = b_range;
  corner_ranges[1] = d_range;
  corner_ranges[2] = range;
  corner_ranges[3] = c_range;
  b = rtIsNaN(b_range);
  if (!b) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 4)) {
      if (!rtIsNaN(corner_ranges[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    range = b_range;
  } else {
    range = corner_ranges[idx - 1];
    idx++;
    for (k = idx; k < 5; k++) {
      v_radial_max = corner_ranges[k - 1];
      if (range < v_radial_max) {
        range = v_radial_max;
      }
    }
  }

  range_max = range + range_gate_abs;
  if (!b) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 4)) {
      if (!rtIsNaN(corner_ranges[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx != 0) {
    b_range = corner_ranges[idx - 1];
    idx++;
    for (k = idx; k < 5; k++) {
      v_radial_max = corner_ranges[k - 1];
      if (b_range > v_radial_max) {
        b_range = v_radial_max;
      }
    }
  }

  range_min = b_range - range_gate_abs;
  if (!(range_min > 0.0)) {
    range_min = 0.0;
  }

  obj->asso.range_interval = range_max - range_min;

  //  angle interval
  corner_ranges[0] = rt_atan2d_snf(c_py_sensor, px_sensor) - obj_angle;
  corner_ranges[1] = rt_atan2d_snf(d_py_sensor, c_px_sensor) - obj_angle;
  corner_ranges[2] = rt_atan2d_snf(b_py_sensor, d_px_sensor) - obj_angle;
  corner_ranges[3] = rt_atan2d_snf(py_sensor, b_px_sensor) - obj_angle;

  //  substitute for matlab's wrapToPi, which is not supported for code
  //  generation
  while (corner_ranges[0] < -3.1415926535897931) {
    corner_ranges[0] += 6.2831853071795862;
  }

  while (corner_ranges[0] > 3.1415926535897931) {
    corner_ranges[0] -= 6.2831853071795862;
  }

  while (corner_ranges[1] < -3.1415926535897931) {
    corner_ranges[1] += 6.2831853071795862;
  }

  while (corner_ranges[1] > 3.1415926535897931) {
    corner_ranges[1] -= 6.2831853071795862;
  }

  while (corner_ranges[2] < -3.1415926535897931) {
    corner_ranges[2] += 6.2831853071795862;
  }

  while (corner_ranges[2] > 3.1415926535897931) {
    corner_ranges[2] -= 6.2831853071795862;
  }

  while (corner_ranges[3] < -3.1415926535897931) {
    corner_ranges[3] += 6.2831853071795862;
  }

  while (corner_ranges[3] > 3.1415926535897931) {
    corner_ranges[3] -= 6.2831853071795862;
  }

  //  transform to obj to avoid 2pi ambiguity
  b = rtIsNaN(corner_ranges[0]);
  if (!b) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 4)) {
      if (!rtIsNaN(corner_ranges[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    range = corner_ranges[0];
  } else {
    range = corner_ranges[idx - 1];
    idx++;
    for (k = idx; k < 5; k++) {
      v_radial_max = corner_ranges[k - 1];
      if (range < v_radial_max) {
        range = v_radial_max;
      }
    }
  }

  angle_max = range + angle_gate_abs;
  if (!b) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 4)) {
      if (!rtIsNaN(corner_ranges[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    range = corner_ranges[0];
  } else {
    range = corner_ranges[idx - 1];
    idx++;
    for (k = idx; k < 5; k++) {
      v_radial_max = corner_ranges[k - 1];
      if (range > v_radial_max) {
        range = v_radial_max;
      }
    }
  }

  b_py_sensor = range - angle_gate_abs;
  obj->asso.ang_interval = angle_max - b_py_sensor;

  //  velocity interval
  corner_ranges[0] = measurement1_idx_2;
  corner_ranges[1] = measurement2_idx_2;
  corner_ranges[2] = measurement3_idx_2;
  corner_ranges[3] = ((state_augmented_idx_1 - vx_ego) * b_px_sensor +
                      state_augmented_idx_4 * py_sensor) / c_range;
  b = rtIsNaN(measurement1_idx_2);
  if (!b) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 4)) {
      if (!rtIsNaN(corner_ranges[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    range = measurement1_idx_2;
  } else {
    range = corner_ranges[idx - 1];
    idx++;
    for (k = idx; k < 5; k++) {
      v_radial_max = corner_ranges[k - 1];
      if (range > v_radial_max) {
        range = v_radial_max;
      }
    }
  }

  py_sensor = range - vr_absGate;
  if (!b) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 4)) {
      if (!rtIsNaN(corner_ranges[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx != 0) {
    measurement1_idx_2 = corner_ranges[idx - 1];
    idx++;
    for (k = idx; k < 5; k++) {
      v_radial_max = corner_ranges[k - 1];
      if (measurement1_idx_2 < v_radial_max) {
        measurement1_idx_2 = v_radial_max;
      }
    }
  }

  v_radial_max = measurement1_idx_2 + vr_absGate;
  obj->asso.vel_interval = v_radial_max - py_sensor;

  //  range Gating
  if ((y[0] < range_max) && (y[0] > range_min)) {
    //  angle Gating
    //  substitute for matlab's wrapToPi, which is not supported for code
    //  generation
    for (range = y[1] - obj_angle; range < -3.1415926535897931; range +=
         6.2831853071795862) {
    }

    while (range > 3.1415926535897931) {
      range -= 6.2831853071795862;
    }

    if ((range > b_py_sensor) && (range < angle_max)) {
      //  vr-Gating
      //  velocity differs for parts of car, when vehicle is nearby (e.g. crossing in front of us) 
      if ((y[2] > py_sensor) && (y[2] < v_radial_max)) {
        y_aso[0] = y[0];
        y_aso[1] = y[1];
        y_aso[2] = y[2];
        *is_associated = true;
        *mark_as_associated = true;
      } else if (std::abs(y[2]) < vr_absGate) {
        //  probably a wheel reflex from floor
        //  not associate, but mark (not generate new objects from it)
        *mark_as_associated = true;
      } else {
        if ((y[2] > 2.0 * py_sensor) && (y[2] < 2.0 * v_radial_max)) {
          //  probably a wheel reflex from top (twice speed)
          //  not associate, but mark (not generate new objects from it)
          *mark_as_associated = true;
        }
      }
    }
  }
}

//
// File trailer for asso_gating.cpp
//
// [EOF]
//
