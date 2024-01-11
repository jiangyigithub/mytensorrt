//
// File: asso_meas2track.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "asso_meas2track.h"
#include "asso_gating.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions

//
// measurement association
//  - Gating (fixed gate size)
// Arguments    : OBJECT_STRUCT *obj
//                MEASLIST_STRUCT *measurements
//                double idx_meas
//                e_struct_T asso_list[20]
// Return Type  : void
//
void asso_meas2track(OBJECT_STRUCT *obj, MEASLIST_STRUCT *measurements, double
                     idx_meas, e_struct_T asso_list[20])
{
  int measurements_tmp;
  double b_measurements[3];
  double c_measurements[2];
  double y_gated[3];
  bool is_associated;
  bool mark_as_associated;
  double y[9];
  static const double varargin_1[9] = { 2.25, 0.0, 0.0, 0.0,
    0.0012184696791468343, 0.0, 0.0, 0.0, 0.16000000000000003 };

  double R[9];

  //  load measurement --------------------------------------------------------- 
  //  original measurement space
  //  gating ------------------------------------------------------------------- 
  measurements_tmp = static_cast<int>(idx_meas) - 1;
  b_measurements[0] = measurements->dr[measurements_tmp];
  b_measurements[1] = measurements->alpha[measurements_tmp];
  b_measurements[2] = measurements->vr[measurements_tmp];
  c_measurements[0] = measurements->dx_sens_offset;
  c_measurements[1] = measurements->dy_sens_offset;
  asso_gating(obj, b_measurements, c_measurements, measurements->vx_ego, y_gated,
              &is_associated, &mark_as_associated);
  if (mark_as_associated || is_associated) {
    measurements->asso[measurements_tmp] = 1.0;
  }

  if (is_associated) {
    int k;
    bool exitg1;

    //  measurement variance
    y[0] = measurements->drVar[measurements_tmp];
    y[3] = 0.0;
    y[6] = 0.0;
    y[1] = 0.0;
    y[4] = measurements->alpVar[measurements_tmp];
    y[7] = 0.0;
    y[2] = 0.0;
    y[5] = 0.0;
    y[8] = measurements->vrVar[measurements_tmp];
    for (k = 0; k < 9; k++) {
      if ((varargin_1[k] > y[k]) || rtIsNaN(y[k])) {
        R[k] = varargin_1[k];
      } else {
        R[k] = y[k];
      }
    }

    obj->meas = true;
    obj->t_lastmeas = measurements->t;
    obj->meas_angles++;

    //  number of associated angles
    //  copy measuremet to association list
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 20)) {
      if (asso_list[k].loc_nr == 0.0) {
        //  free entry
        asso_list[k].y[0] = y_gated[0];
        asso_list[k].y[1] = y_gated[1];
        asso_list[k].y[2] = y_gated[2];
        std::memcpy(&asso_list[k].R[0], &R[0], 9U * sizeof(double));
        asso_list[k].PDH1 = measurements->PDH1[measurements_tmp];
        asso_list[k].PDH0 = measurements->PDH0[measurements_tmp];
        asso_list[k].potGhost = measurements->potGhost[measurements_tmp];
        asso_list[k].loc_nr = idx_meas;
        exitg1 = true;
      } else {
        k++;
      }
    }

    //  if list is full, do not use measurement
  } else {
    //  gating failed
  }
}

//
// File trailer for asso_meas2track.cpp
//
// [EOF]
//
