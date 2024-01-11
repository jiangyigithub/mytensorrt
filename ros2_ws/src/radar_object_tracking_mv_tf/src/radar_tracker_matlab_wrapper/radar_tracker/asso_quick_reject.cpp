//
// File: asso_quick_reject.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "asso_quick_reject.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// discard potential ghost targets
// Arguments    : double obj_pexist
//                const double obj_x[6]
//                double obj_length
//                double obj_width
//                const double meas_list_potDoubleRefl[512]
//                const double meas_list_meas[512]
//                const double meas_list_dr[512]
//                double meas_list_dx_sens_offset
//                double meas_list_dy_sens_offset
//                bool meas_reject[512]
// Return Type  : void
//
void asso_quick_reject(double obj_pexist, const double obj_x[6], double
  obj_length, double obj_width, const double meas_list_potDoubleRefl[512], const
  double meas_list_meas[512], const double meas_list_dr[512], double
  meas_list_dx_sens_offset, double meas_list_dy_sens_offset, bool meas_reject
  [512])
{
  bool b;
  double longer_edge;
  double scale;
  double absxk;
  double t;
  double dr_obj_max_tmp;
  b = (obj_pexist < 0.8);

  //  Load range meas value
  //  Quick Reject -----------
  //  rough range gating
  if ((obj_length > obj_width) || rtIsNaN(obj_width)) {
    longer_edge = obj_length;
  } else {
    longer_edge = obj_width;
  }

  scale = 3.3121686421112381E-170;
  absxk = std::abs(obj_x[0] - meas_list_dx_sens_offset);
  if (absxk > 3.3121686421112381E-170) {
    dr_obj_max_tmp = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    dr_obj_max_tmp = t * t;
  }

  absxk = std::abs(obj_x[3] - meas_list_dy_sens_offset);
  if (absxk > scale) {
    t = scale / absxk;
    dr_obj_max_tmp = dr_obj_max_tmp * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    dr_obj_max_tmp += t * t;
  }

  dr_obj_max_tmp = scale * std::sqrt(dr_obj_max_tmp);
  absxk = dr_obj_max_tmp + longer_edge;
  scale = dr_obj_max_tmp - longer_edge;
  for (int i = 0; i < 512; i++) {
    meas_reject[i] = ((meas_list_meas[i] == 0.0) || (b &&
      (meas_list_potDoubleRefl[i] == 1.0)) || (meas_list_dr[i] < scale - 5.0) ||
                      (meas_list_dr[i] > absxk + 5.0));
  }
}

//
// File trailer for asso_quick_reject.cpp
//
// [EOF]
//
