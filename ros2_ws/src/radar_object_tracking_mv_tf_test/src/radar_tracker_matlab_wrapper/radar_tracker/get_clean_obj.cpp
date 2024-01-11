//
// File: get_clean_obj.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "get_clean_obj.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions

//
// Arguments    : OBJECT_STRUCT *obj
// Return Type  : void
//
void get_clean_obj(OBJECT_STRUCT *obj)
{
  int i;
  struct2_T s;
  obj->valid = false;
  obj->meas = false;
  obj->meas_angles = 0.0;

  //  counter for associated angles
  obj->measureable = false;
  obj->hist = false;
  obj->age = 0.0;
  obj->t_abs = 0.0;
  obj->t_lastmeas = 0.0;
  obj->standing = 0.0;

  //  probability
  obj->moving = 0.0;

  //  probability
  obj->pexist = 0.0;

  //  probability
  obj->Pgr_reflex = 0.0;

  //  Guardrail Reflex
  for (i = 0; i < 6; i++) {
    obj->x[i] = 0.0;
  }

  std::memset(&obj->P[0], 0, 36U * sizeof(double));
  obj->length = 0.0;
  obj->width = 0.0;
  obj->psiDt = 0.0;
  obj->psi = 0.0;
  obj->P_psi[0] = 0.0;
  obj->P_psi[1] = 0.0;
  obj->P_psi[2] = 0.0;
  obj->P_psi[3] = 0.0;
  obj->RCS_filt = 0.0;
  obj->P_obj_type[0] = 0.0;
  obj->P_obj_type[1] = 0.0;
  obj->length_meas = 0.0;
  obj->length_filt = 0.0;
  obj->width_meas = 0.0;
  obj->width_filt = 0.0;

  //  historical trajectory (world coordinates)
  std::memset(&obj->traj_hist_x[0], 0, 20U * sizeof(double));
  std::memset(&obj->traj_hist_y[0], 0, 20U * sizeof(double));
  std::memset(&obj->traj_hist_vx[0], 0, 20U * sizeof(double));
  std::memset(&obj->traj_hist_vy[0], 0, 20U * sizeof(double));
  std::memset(&obj->traj_hist_t[0], 0, 20U * sizeof(double));
  obj->traj_hist_len = 0.0;
  obj->psi_traj_hist = 0.0;
  obj->grid.step = 0.5;
  for (i = 0; i < 21; i++) {
    double d;
    d = 0.5 * static_cast<double>(i) + -5.0;
    obj->grid.x_cells[i] = d;
    obj->grid.y_cells[i] = d;
  }

  obj->grid.x_len = 21.0;
  obj->grid.y_len = 21.0;
  std::memset(&obj->grid.LR[0], 0, 441U * sizeof(double));
  obj->asso.range_interval = 0.0;
  obj->asso.vel_interval = 0.0;
  obj->asso.ang_interval = 0.0;
  for (i = 0; i < 6; i++) {
    s.x[i] = 0.0;
  }

  std::memset(&s.P[0], 0, 36U * sizeof(double));
  s.mu = 0.33333333333333331;
  s.pdf_max = 0.0;
  obj->IMM[0] = s;
  obj->IMM[1] = s;
  obj->IMM[2] = s;
}

//
// File trailer for get_clean_obj.cpp
//
// [EOF]
//
