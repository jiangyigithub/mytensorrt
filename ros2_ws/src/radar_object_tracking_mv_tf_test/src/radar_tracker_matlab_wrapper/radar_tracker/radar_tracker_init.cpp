//
// File: radar_tracker_init.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "radar_tracker_init.h"
#include "radar_tracker.h"
#include "radar_tracker_data.h"
#include "radar_tracker_initialize.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions

//
// Objektliste initialisieren
// Arguments    : OBJECT_STRUCT obj_list[50]
// Return Type  : void
//
void radar_tracker_init(OBJECT_STRUCT obj_list[50])
{
  OBJECT_STRUCT s;
  int i;
  struct2_T b_s;
  if (!isInitialized_radar_tracker) {
    radar_tracker_initialize();
  }

  s.valid = false;
  s.meas = false;
  s.meas_angles = 0.0;
  s.measureable = false;
  s.hist = false;
  s.age = 0.0;
  s.t_abs = 0.0;
  s.t_lastmeas = 0.0;
  s.standing = 0.0;
  s.moving = 0.0;
  s.pexist = 0.0;
  s.Pgr_reflex = 0.0;
  for (i = 0; i < 6; i++) {
    s.x[i] = 0.0;
  }

  std::memset(&s.P[0], 0, 36U * sizeof(double));
  s.length = 0.0;
  s.width = 0.0;
  s.psiDt = 0.0;
  s.psi = 0.0;
  s.P_psi[0] = 0.0;
  s.P_psi[1] = 0.0;
  s.P_psi[2] = 0.0;
  s.P_psi[3] = 0.0;
  s.RCS_filt = 0.0;
  s.P_obj_type[0] = 0.0;
  s.P_obj_type[1] = 0.0;
  s.length_meas = 0.0;
  s.length_filt = 0.0;
  s.width_meas = 0.0;
  s.width_filt = 0.0;
  std::memset(&s.traj_hist_x[0], 0, 20U * sizeof(double));
  std::memset(&s.traj_hist_y[0], 0, 20U * sizeof(double));
  std::memset(&s.traj_hist_vx[0], 0, 20U * sizeof(double));
  std::memset(&s.traj_hist_vy[0], 0, 20U * sizeof(double));
  std::memset(&s.traj_hist_t[0], 0, 20U * sizeof(double));
  s.traj_hist_len = 0.0;
  s.psi_traj_hist = 0.0;
  s.grid.step = 0.5;
  for (i = 0; i < 21; i++) {
    double d;
    d = 0.5 * static_cast<double>(i) + -5.0;
    s.grid.x_cells[i] = d;
    s.grid.y_cells[i] = d;
  }

  s.grid.x_len = 21.0;
  s.grid.y_len = 21.0;
  std::memset(&s.grid.LR[0], 0, 441U * sizeof(double));
  s.asso.range_interval = 0.0;
  s.asso.vel_interval = 0.0;
  s.asso.ang_interval = 0.0;
  for (i = 0; i < 6; i++) {
    b_s.x[i] = 0.0;
  }

  std::memset(&b_s.P[0], 0, 36U * sizeof(double));
  b_s.mu = 0.33333333333333331;
  b_s.pdf_max = 0.0;
  s.IMM[0] = b_s;
  s.IMM[1] = b_s;
  s.IMM[2] = b_s;
  for (i = 0; i < 50; i++) {
    obj_list[i] = s;
  }
}

//
// File trailer for radar_tracker_init.cpp
//
// [EOF]
//
