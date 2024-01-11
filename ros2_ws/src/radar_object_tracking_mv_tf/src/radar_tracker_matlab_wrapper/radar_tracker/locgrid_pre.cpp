//
// File: locgrid_pre.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "locgrid_pre.h"
#include "mgmt_init_track.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// LOCGRID_PRE is the predict step of the local grd map.
//  This is basically only a forgetting functionality of the LR intesity
// Arguments    : OBJECT_STRUCT *obj
//                double dt
// Return Type  : void
//
void locgrid_pre(OBJECT_STRUCT *obj, double dt)
{
  double u1;
  u1 = 0.3 * std::sqrt(obj->x[0] * obj->x[0] + obj->x[3] * obj->x[3]) / 40.0;
  if ((0.3 < u1) || rtIsNaN(u1)) {
    u1 = 0.3;
  }

  u1 = rt_powd_snf(0.97 * u1, dt);

  //  limit upper/lower bound
  //  probability 0.999
  for (int k = 0; k < 441; k++) {
    double u0;
    obj->grid.LR[k] *= u1;
    u0 = obj->grid.LR[k];
    if (!(u0 < 1000.0)) {
      u0 = 1000.0;
    }

    obj->grid.LR[k] = u0;
    u0 = obj->grid.LR[k];
    if (!(u0 > 0.1111111111111111)) {
      u0 = 0.1111111111111111;
    }

    obj->grid.LR[k] = u0;
  }

  //  probability 0.1
}

//
// File trailer for locgrid_pre.cpp
//
// [EOF]
//
