//
// File: preproc_flagsinit.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "preproc_flagsinit.h"
#include "mgmt_init_track.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// Arguments    : OBJECT_STRUCT *obj
//                double meas_list_t
//                double meas_list_dx_sens_offset
//                double meas_list_dy_sens_offset
//                double meas_list_ang_sens_offset
//                const double meas_list_fov_range[25]
//                const double meas_list_fov_angle[25]
//                double dt
// Return Type  : void
//
void preproc_flagsinit(OBJECT_STRUCT *obj, double meas_list_t, double
  meas_list_dx_sens_offset, double meas_list_dy_sens_offset, double
  meas_list_ang_sens_offset, const double meas_list_fov_range[25], const double
  meas_list_fov_angle[25], double dt)
{
  double obj_px_no_offset;
  double obj_py_no_offset;
  double obj_r_sens;
  bool x[25];
  signed char ii_data[1];
  signed char b_ii_data[1];
  double varargin_1_data[1];
  double varargin_2_data[1];
  double fov_r_interp_data[1];
  bool x_data[1];
  obj->meas = false;
  obj->hist = true;
  obj->meas_angles = 0.0;

  //  Is Object in sensor FOV? -> obj.measureable
  obj_px_no_offset = obj->x[0] - meas_list_dx_sens_offset;

  //  still in vehicle CoSy orientation
  obj_py_no_offset = obj->x[3] - meas_list_dy_sens_offset;

  //  4-quadrant tan (atan2) already considers pi/2 ambiguity of tan; value in [-pi,pi]  
  obj_r_sens = std::sqrt(obj_px_no_offset * obj_px_no_offset + obj_py_no_offset *
    obj_py_no_offset);
  obj_px_no_offset = rt_atan2d_snf(obj_py_no_offset, obj_px_no_offset) -
    meas_list_ang_sens_offset;
  if (obj_px_no_offset < -3.1415926535897931) {
    obj_px_no_offset += 6.2831853071795862;
  } else {
    if (obj_px_no_offset > 3.1415926535897931) {
      obj_px_no_offset -= 6.2831853071795862;
    }
  }

  if ((obj_px_no_offset < meas_list_fov_angle[0]) || (obj_px_no_offset >
       meas_list_fov_angle[24])) {
    //  Object out of FOV bounds
    obj->measureable = false;
  } else {
    int idx;
    int ii_size_idx_1;
    int ii;
    bool exitg1;
    int b_ii_size_idx_1;
    signed char csz_idx_1;
    bool y;
    for (idx = 0; idx < 25; idx++) {
      x[idx] = (obj_px_no_offset > meas_list_fov_angle[idx]);
    }

    idx = 0;
    ii_size_idx_1 = 1;
    ii = 25;
    exitg1 = false;
    while ((!exitg1) && (ii > 0)) {
      if (x[ii - 1]) {
        idx = 1;
        ii_data[0] = static_cast<signed char>(ii);
        exitg1 = true;
      } else {
        ii--;
      }
    }

    if (idx == 0) {
      ii_size_idx_1 = 0;
    }

    for (idx = 0; idx < 25; idx++) {
      x[idx] = (obj_px_no_offset < meas_list_fov_angle[idx]);
    }

    idx = 0;
    b_ii_size_idx_1 = 1;
    ii = 0;
    exitg1 = false;
    while ((!exitg1) && (ii < 25)) {
      if (x[ii]) {
        idx = 1;
        b_ii_data[0] = static_cast<signed char>(ii + 1);
        exitg1 = true;
      } else {
        ii++;
      }
    }

    if (idx == 0) {
      b_ii_size_idx_1 = 0;
    }

    //      fov_r_interp = interpolate_2points(meas_list.fov_angle(fov_sec_index_lower), ... 
    //                                         meas_list.fov_angle(fov_sec_index_higher), ... 
    //                                         meas_list.fov_range(fov_sec_index_lower), ... 
    //                                         meas_list.fov_range(fov_sec_index_higher), ... 
    //                                         obj_ang_sens);
    if (0 <= ii_size_idx_1 - 1) {
      varargin_1_data[0] = meas_list_fov_range[ii_data[0] - 1];
    }

    if (0 <= b_ii_size_idx_1 - 1) {
      varargin_2_data[0] = meas_list_fov_range[b_ii_data[0] - 1];
    }

    if (ii_size_idx_1 <= b_ii_size_idx_1) {
      csz_idx_1 = static_cast<signed char>(ii_size_idx_1);
    } else {
      csz_idx_1 = 0;
      ii_size_idx_1 = 0;
    }

    if (0 <= ii_size_idx_1 - 1) {
      if ((varargin_1_data[0] > varargin_2_data[0]) || rtIsNaN(varargin_2_data[0]))
      {
        fov_r_interp_data[0] = varargin_1_data[0];
      } else {
        fov_r_interp_data[0] = varargin_2_data[0];
      }
    }

    idx = csz_idx_1;
    if (0 <= idx - 1) {
      x_data[0] = (obj_r_sens < fov_r_interp_data[0]);
    }

    y = (csz_idx_1 != 0);
    if (y) {
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx <= csz_idx_1 - 1)) {
        if (!x_data[0]) {
          y = false;
          exitg1 = true;
        } else {
          idx = 1;
        }
      }
    }

    obj->measureable = y;
  }

  obj->IMM[0].pdf_max = 0.0;
  obj->IMM[1].pdf_max = 0.0;
  obj->IMM[2].pdf_max = 0.0;
  if (dt > 0.0) {
    obj->age += dt;
  }

  //  update timestamp
  obj->t_abs = meas_list_t;
}

//
// File trailer for preproc_flagsinit.cpp
//
// [EOF]
//
