//
// File: mgmt_init_track.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "mgmt_init_track.h"
#include "imm_output.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

//lnl, debugging
#ifndef tracker_initial
#define tracker_initial false
#endif
#include <iostream>

// Type Definitions
struct struct_T
{
  double r[5];
  double v[5];
  double azimuth_global[5];
  double num_locs_observed;
  double last_update_age;
  double dt[5];
};

struct b_struct_T
{
  double r[5];
  double v[5];
  double azimuth_global[5];
  double dt[5];
  double r0;
  double v0;
  double alpha0;
  double psi0;
  double var_r0;
  double var_v0;
  double var_alpha0;
};

struct c_struct_T
{
  double r;
  double v;
  double azimuth_global;
  double range_gate;
  double vel_gate;
  double ang_gate;
};

// Variable Definitions
static struct_T locations_store[1000];

// Function Declarations
static void merge_tracks_init(const b_struct_T new_track_candidates[100], double
  num_new_tracks, bool new_tracks_valid[100]);
static void new_track_regression(b_struct_T new_track_candidates[100], double
  num_new_tracks);
static void set_new_track_properties(OBJECT_STRUCT obj[50], const b_struct_T
  new_track_candidates[100], double num_new_tracks, const bool new_tracks_valid
  [100], double vx_ego, double current_time);
static void trackletManagementInit(const double measurements_asso[512], const
  double measurements_potDoubleRefl[512], const double measurements_meas[512],
  const double measurements_dr[512], const double measurements_vr[512], const
  double measurements_dBRcs[512], double measurements_dx_sens_offset, double
  measurements_dy_sens_offset, const double measurements_alpha[512], double
  vx_ego, double dt, b_struct_T new_track_candidates[100], double
  *num_new_tracks);

// Function Definitions

//
// defs
// Arguments    : const b_struct_T new_track_candidates[100]
//                double num_new_tracks
//                bool new_tracks_valid[100]
// Return Type  : void
//
static void merge_tracks_init(const b_struct_T new_track_candidates[100], double
  num_new_tracks, bool new_tracks_valid[100])
{
  bool track_match[10000];
  int loop_ub;
  int i2;
  int iy;
  bool new_tracks_merge[100];
  int idx2;
  int a;

  //  m
  //  m/s
  //  3 degree
  //  merge
  std::memset(&track_match[0], 0, 10000U * sizeof(bool));

  //  fixed size for coder not to use problematic memset
  loop_ub = static_cast<int>(num_new_tracks);
  for (i2 = 0; i2 < loop_ub; i2++) {
    iy = static_cast<int>(num_new_tracks + (1.0 - ((static_cast<double>(i2) +
      1.0) + 1.0)));
    for (idx2 = 0; idx2 < iy; idx2++) {
      a = static_cast<int>(((static_cast<double>(i2) + 1.0) + 1.0) +
                           static_cast<double>(idx2)) - 1;
      if ((std::abs(new_track_candidates[i2].r0 - new_track_candidates[a].r0) <
           3.0) && (std::abs(new_track_candidates[i2].v0 -
                             new_track_candidates[a].v0) < 2.0) && (std::abs
           (new_track_candidates[i2].alpha0 - new_track_candidates[a].alpha0) <
           0.052359877559829883)) {
        track_match[i2 + 100 * a] = true;
      }
    }
  }

  std::memset(&new_tracks_valid[0], 0, 100U * sizeof(bool));
  std::memset(&new_tracks_merge[0], 0, 100U * sizeof(bool));
  i2 = 1;
  iy = -1;
  for (idx2 = 0; idx2 < 100; idx2++) {
    int ix;
    bool exitg1;
    a = i2 + 99;
    ix = i2;
    i2 += 100;
    iy++;
    exitg1 = false;
    while ((!exitg1) && (ix <= a)) {
      if (track_match[ix - 1]) {
        new_tracks_merge[iy] = true;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }

  for (iy = 0; iy < loop_ub; iy++) {
    new_tracks_valid[iy] = !new_tracks_merge[iy];
  }
}

//
// Arguments    : b_struct_T new_track_candidates[100]
//                double num_new_tracks
// Return Type  : void
//
static void new_track_regression(b_struct_T new_track_candidates[100], double
  num_new_tracks)
{
  int i;
  double meas_age[5];
  double dy[5];
  double dx[5];
  double dx_tmp;
  double dv[25];
  double b_dx;
  double d2;
  i = static_cast<int>(num_new_tracks);
  for (int track_ind = 0; track_ind < i; track_ind++) {
    int k;
    double psi_idx_0;
    double psi_idx_1;
    double psi_idx_2;
    double psi_idx_3;
    double d;
    double d1;
    int i1;
    meas_age[0] = 0.0;
    meas_age[1] = new_track_candidates[track_ind].dt[4];
    meas_age[2] = new_track_candidates[track_ind].dt[3];
    meas_age[2] += meas_age[1];
    meas_age[3] = new_track_candidates[track_ind].dt[2];
    meas_age[3] += meas_age[2];
    meas_age[4] = new_track_candidates[track_ind].dt[1];
    meas_age[4] += meas_age[3];

    #if tracker_initial
    std::cout<<" 66_regression_ meas_age[4]: "<<meas_age[4]<<std::endl;
    #endif

    //  dx/dy only to calculate heading psi0
    for (k = 0; k < 5; k++) {
      dx_tmp = new_track_candidates[track_ind].azimuth_global[4 - k];
      psi_idx_0 = new_track_candidates[track_ind].r[4 - k];
      dx[k] = psi_idx_0 * std::cos(dx_tmp);
      dy[k] = psi_idx_0 * std::sin(dx_tmp);
    }

    psi_idx_0 = rt_atan2d_snf(dy[4] - dy[3], dx[4] - dx[3]);
    psi_idx_1 = rt_atan2d_snf(dy[3] - dy[2], dx[3] - dx[2]);
    psi_idx_2 = rt_atan2d_snf(dy[2] - dy[1], dx[2] - dx[1]);
    psi_idx_3 = rt_atan2d_snf(dy[1] - dy[0], dx[1] - dx[0]);

    //  weights for LS fit; older measurements receive less weight
    dx_tmp = 0.0;
    for (k = 0; k < 5; k++) {
      d = std::exp(-new_track_candidates[track_ind].dt[k] / 0.25);
      dy[k] = d;
      dx_tmp += new_track_candidates[track_ind].azimuth_global[4 - k] * d;
    }

    d = ((dy[0] + dy[1]) + dy[2]) + dy[3];
    d1 = d + dy[4];
    new_track_candidates[track_ind].alpha0 = dx_tmp / d1;

    //  weighted sample mean
    //  substitute for matlab's wrapToPi, which is not supported for code
    //  generation
    for (k = 0; k < 5; k++) {
      dx[k] = new_track_candidates[track_ind].azimuth_global[4 - k] -
        new_track_candidates[track_ind].alpha0;
      while (dx[k] < -3.1415926535897931) {
        dx[k] += 6.2831853071795862;
      }

      while (dx[k] > 3.1415926535897931) {
        dx[k] -= 6.2831853071795862;
      }
    }

    std::memset(&dv[0], 0, 25U * sizeof(double));
    for (k = 0; k < 5; k++) {
      dv[k + 5 * k] = dy[k];
    }

    b_dx = 0.0;

    //  weighted sample variance
    dx_tmp = 0.0;
    for (k = 0; k < 5; k++) {
      d2 = 0.0;
      for (i1 = 0; i1 < 5; i1++) {
        d2 += dx[i1] * dv[i1 + 5 * k];
      }

      b_dx += d2 * dx[k];
      dx_tmp += new_track_candidates[track_ind].v[4 - k] * dy[k];
    }

    new_track_candidates[track_ind].var_alpha0 = b_dx / d1;
    new_track_candidates[track_ind].v0 = dx_tmp / d1;

    //  weighted sample mean
    for (k = 0; k < 5; k++) {
      dx[k] = new_track_candidates[track_ind].v[4 - k] -
        new_track_candidates[track_ind].v0;
    }

    b_dx = 0.0;

    //  weighted sample variance
    dx_tmp = 0.0;
    for (k = 0; k < 5; k++) {
      d2 = 0.0;
      for (i1 = 0; i1 < 5; i1++) {
        d2 += dx[i1] * dv[i1 + 5 * k];
      }

      b_dx += d2 * dx[k];
      d2 = new_track_candidates[track_ind].r[4 - k] + meas_age[k] *
        new_track_candidates[track_ind].v0;
      meas_age[k] = d2;
      dx_tmp += d2 * dy[k];
    }

    new_track_candidates[track_ind].var_v0 = b_dx / d1;
    new_track_candidates[track_ind].r0 = dx_tmp / d1;

    //  weighted sample mean
    for (k = 0; k < 5; k++) {
      meas_age[k] -= new_track_candidates[track_ind].r0;
    }

    b_dx = 0.0;
    for (k = 0; k < 5; k++) {
      d2 = 0.0;
      for (i1 = 0; i1 < 5; i1++) {
        d2 += meas_age[i1] * dv[i1 + 5 * k];
      }

      b_dx += d2 * meas_age[k];
    }

    new_track_candidates[track_ind].var_r0 = b_dx / d1;

    //  weighted sample variance
    new_track_candidates[track_ind].psi0 = (((psi_idx_0 * dy[0] + psi_idx_1 *
      dy[1]) + psi_idx_2 * dy[2]) + psi_idx_3 * dy[3]) / d;
  }
}

//
// Arguments    : OBJECT_STRUCT obj[50]
//                const b_struct_T new_track_candidates[100]
//                double num_new_tracks
//                const bool new_tracks_valid[100]
//                double vx_ego
//                double current_time
// Return Type  : void
//
static void set_new_track_properties(OBJECT_STRUCT obj[50], const b_struct_T
  new_track_candidates[100], double num_new_tracks, const bool new_tracks_valid
  [100], double vx_ego, double current_time)
{
  int i;
  double dr2_stat_max;
  double s_grid_x_cells[21];
  struct2_T s;
  double s_grid_y_cells[21];
  OBJECT_STRUCT expl_temp;
  double gs[16];
  double Rs[16];
  double b_gs[16];
  double R_final[36];
  static const short iv[6] = { 0, 0, 1000, 0, 0, 0 };

  static const short iv1[6] = { 0, 0, 0, 0, 0, 1000 };

  static const double dv[6] = { 0.0, 0.0, 0.030461741978670857, 0.0, 0.0, 0.0 };

  static const signed char iv2[6] = { 0, 0, 0, 1, 0, 0 };

  static const short iv3[6] = { 0, 0, 0, 0, 1000, 0 };

  i = static_cast<int>(num_new_tracks);
  for (int track_ind = 0; track_ind < i; track_ind++) {
    if (new_tracks_valid[track_ind]) {
      int new_obj_idx;
      int b_i;
      bool exitg1;
      double dr2_stat;

      //  search for free object
      new_obj_idx = -1;
      b_i = 0;
      exitg1 = false;
      while ((!exitg1) && (b_i < 50)) {
        if (!obj[b_i].valid) {
          new_obj_idx = b_i;
          exitg1 = true;
        } else {
          b_i++;
        }
      }

      //  if object list full and we have a moving object, then delete most distant stationary target 
      if ((new_obj_idx + 1 == 0) && (std::abs(new_track_candidates[track_ind].v0)
           > 1.0)) {
        dr2_stat_max = 0.0;
        for (b_i = 0; b_i < 50; b_i++) {
          if (obj[b_i].valid && (obj[b_i].standing > 0.95)) {
            dr2_stat = obj[b_i].x[0] * obj[b_i].x[0] + obj[b_i].x[3] * obj[b_i].
              x[3];
            if (dr2_stat > dr2_stat_max) {
              dr2_stat_max = dr2_stat;
              new_obj_idx = b_i;
            }
          }
        }
      }

      if (new_obj_idx + 1 > 0) {
        int i1;

        //  empty object overwrite (to be on the save side)
        for (i1 = 0; i1 < 21; i1++) {
          dr2_stat_max = 0.5 * static_cast<double>(i1) + -5.0;
          s_grid_x_cells[i1] = dr2_stat_max;
          s_grid_y_cells[i1] = dr2_stat_max;
        }

        for (b_i = 0; b_i < 6; b_i++) {
          s.x[b_i] = 0.0;
        }

        std::memset(&s.P[0], 0, 36U * sizeof(double));
        s.mu = 0.33333333333333331;
        s.pdf_max = 0.0;
        expl_temp.valid = false;
        expl_temp.meas = false;
        expl_temp.meas_angles = 0.0;
        expl_temp.measureable = false;
        expl_temp.hist = false;
        expl_temp.age = 0.0;
        expl_temp.t_abs = 0.0;
        expl_temp.t_lastmeas = 0.0;
        expl_temp.standing = 0.0;
        expl_temp.moving = 0.0;
        expl_temp.pexist = 0.0;
        expl_temp.Pgr_reflex = 0.0;
        for (b_i = 0; b_i < 6; b_i++) {
          expl_temp.x[b_i] = 0.0;
        }

        std::memset(&expl_temp.P[0], 0, 36U * sizeof(double));
        expl_temp.length = 0.0;
        expl_temp.width = 0.0;
        expl_temp.psiDt = 0.0;
        expl_temp.psi = 0.0;
        expl_temp.P_psi[0] = 0.0;
        expl_temp.P_psi[1] = 0.0;
        expl_temp.P_psi[2] = 0.0;
        expl_temp.P_psi[3] = 0.0;
        expl_temp.RCS_filt = 0.0;
        expl_temp.P_obj_type[0] = 0.0;
        expl_temp.P_obj_type[1] = 0.0;
        expl_temp.length_meas = 0.0;
        expl_temp.length_filt = 0.0;
        expl_temp.width_meas = 0.0;
        expl_temp.width_filt = 0.0;
        std::memset(&expl_temp.traj_hist_x[0], 0, 20U * sizeof(double));
        std::memset(&expl_temp.traj_hist_y[0], 0, 20U * sizeof(double));
        std::memset(&expl_temp.traj_hist_vx[0], 0, 20U * sizeof(double));
        std::memset(&expl_temp.traj_hist_vy[0], 0, 20U * sizeof(double));
        std::memset(&expl_temp.traj_hist_t[0], 0, 20U * sizeof(double));
        expl_temp.traj_hist_len = 0.0;
        expl_temp.psi_traj_hist = 0.0;
        expl_temp.grid.step = 0.5;
        std::memcpy(&expl_temp.grid.x_cells[0], &s_grid_x_cells[0], 21U * sizeof
                    (double));
        std::memcpy(&expl_temp.grid.y_cells[0], &s_grid_y_cells[0], 21U * sizeof
                    (double));
        expl_temp.grid.x_len = 21.0;
        expl_temp.grid.y_len = 21.0;
        std::memset(&expl_temp.grid.LR[0], 0, 441U * sizeof(double));
        expl_temp.asso.range_interval = 0.0;
        expl_temp.asso.vel_interval = 0.0;
        expl_temp.asso.ang_interval = 0.0;
        expl_temp.IMM[0] = s;
        expl_temp.IMM[1] = s;
        expl_temp.IMM[2] = s;
        obj[new_obj_idx] = expl_temp;
        dr2_stat_max = std::abs(new_track_candidates[track_ind].v0);
        if (!(dr2_stat_max < 0.5)) {
          double x;
          double dx_tmp;
          double dx;
          double dy_tmp;
          double dy;
          double psi;
          double dr;
          double vr;
          int i2;

          //  heuristic: velocity threshold for 0.5 m/s, then exp behavior
          //  (P=0.7 at 1m/s for vx_ego=0, P=0.6 at 1m/s for vx_ego=2.7 (Shuttle speed) 
          dr2_stat_max -= 0.5;
          if (!(dr2_stat_max > 0.0)) {
            dr2_stat_max = 0.0;
          }

          x = std::exp(-0.5 * dr2_stat_max / (0.02 * std::abs(vx_ego) + 0.2));
          dx_tmp = std::cos(new_track_candidates[track_ind].alpha0);
          dx = new_track_candidates[track_ind].r0 * dx_tmp;

          //  r is already in global CoSy
          dy_tmp = std::sin(new_track_candidates[track_ind].alpha0);
          dy = new_track_candidates[track_ind].r0 * dy_tmp;

          //  Initialize in our direction
          dr2_stat_max = new_track_candidates[track_ind].v0 * dx_tmp;
          dr2_stat = new_track_candidates[track_ind].v0 * dy_tmp;

          //  object orientation
          psi = rt_atan2d_snf(dr2_stat, dr2_stat_max);
          obj[new_obj_idx].psi = psi;

          //  state vector
          obj[new_obj_idx].x[0] = dx;
          obj[new_obj_idx].x[1] = dr2_stat_max;
          obj[new_obj_idx].x[2] = 0.0;
          obj[new_obj_idx].x[3] = dy;
          obj[new_obj_idx].x[4] = dr2_stat;
          obj[new_obj_idx].x[5] = 0.0;

          //  transform measurement covariances:  y = [dr vr alpha]'
          //  high uncertainty for direction!
          dr = std::sqrt(dx * dx + dy * dy);
          vr = std::sqrt(dr2_stat_max * dr2_stat_max + dr2_stat * dr2_stat);

          // no tangential velocity
          gs[0] = dx_tmp;
          gs[4] = 0.0;
          gs[8] = 0.0;
          gs[12] = -dr * dy_tmp;
          gs[1] = 0.0;
          gs[5] = dx_tmp;
          gs[9] = -dy_tmp;
          gs[13] = -(vr * dy_tmp + 0.0 * dx_tmp);
          gs[2] = dy_tmp;
          gs[6] = 0.0;
          gs[10] = 0.0;
          gs[14] = dr * dx_tmp;
          gs[3] = 0.0;
          gs[7] = dy_tmp;
          gs[11] = dx_tmp;
          gs[15] = vr * dx_tmp - 0.0 * dy_tmp;

          //  transform covariance
          Rs[0] = new_track_candidates[track_ind].var_r0 + 0.25;
          Rs[4] = 0.0;
          Rs[8] = 0.0;
          Rs[12] = 0.0;
          Rs[1] = 0.0;
          Rs[5] = new_track_candidates[track_ind].var_v0 + 1.0;
          Rs[9] = 0.0;
          Rs[13] = 0.0;
          Rs[2] = 0.0;
          Rs[6] = 0.0;
          Rs[10] = 1000.0;
          Rs[14] = 0.0;
          Rs[3] = 0.0;
          Rs[7] = 0.0;
          Rs[11] = 0.0;
          Rs[15] = new_track_candidates[track_ind].var_alpha0 +
            1.9038588736669286E-5;
          for (i1 = 0; i1 < 4; i1++) {
            dr2_stat_max = gs[i1 + 4];
            dr2_stat = gs[i1 + 8];
            dx_tmp = gs[i1 + 12];
            for (i2 = 0; i2 < 4; i2++) {
              b_i = i2 << 2;
              b_gs[i1 + b_i] = ((gs[i1] * Rs[b_i] + dr2_stat_max * Rs[b_i + 1])
                                + dr2_stat * Rs[b_i + 2]) + dx_tmp * Rs[b_i + 3];
            }
          }

          for (i1 = 0; i1 < 4; i1++) {
            dr2_stat_max = b_gs[i1 + 4];
            dr2_stat = b_gs[i1 + 8];
            dx_tmp = b_gs[i1 + 12];
            for (i2 = 0; i2 < 4; i2++) {
              Rs[i1 + (i2 << 2)] = ((b_gs[i1] * gs[i2] + dr2_stat_max * gs[i2 +
                4]) + dr2_stat * gs[i2 + 8]) + dx_tmp * gs[i2 + 12];
            }
          }

          //  consider object extent
          Rs[0] += 0.3;

          // 1.0;
          Rs[10] += 0.3;

          // 1.0;
          R_final[0] = Rs[0];
          R_final[6] = Rs[4];
          R_final[12] = 0.0;
          R_final[18] = Rs[8];
          R_final[24] = Rs[12];
          R_final[30] = 0.0;
          R_final[1] = Rs[1];
          R_final[7] = Rs[5];
          R_final[13] = 0.0;
          R_final[19] = Rs[9];
          R_final[25] = Rs[13];
          R_final[31] = 0.0;
          R_final[3] = Rs[2];
          R_final[9] = Rs[6];
          R_final[15] = 0.0;
          R_final[21] = Rs[10];
          R_final[27] = Rs[14];
          R_final[33] = 0.0;
          R_final[4] = Rs[3];
          R_final[10] = Rs[7];
          R_final[16] = 0.0;
          R_final[22] = Rs[11];
          R_final[28] = Rs[15];
          R_final[34] = 0.0;
          for (i1 = 0; i1 < 6; i1++) {
            R_final[6 * i1 + 2] = iv[i1];
            R_final[6 * i1 + 5] = iv1[i1];
          }

          std::memcpy(&obj[new_obj_idx].P[0], &R_final[0], 36U * sizeof(double));
          obj[new_obj_idx].valid = true;
          obj[new_obj_idx].meas = true;
          obj[new_obj_idx].hist = false;
          obj[new_obj_idx].age = 0.0;
          obj[new_obj_idx].t_abs = current_time;
          obj[new_obj_idx].t_lastmeas = current_time;
          obj[new_obj_idx].standing = 1.0 - (1.0 - x);
          obj[new_obj_idx].moving = 1.0 - x;
          obj[new_obj_idx].pexist = 0.2;
          obj[new_obj_idx].length = 1.0;
          obj[new_obj_idx].width = 1.0;
          obj[new_obj_idx].psiDt = 0.0;
          obj[new_obj_idx].P_psi[0] = 9.869604401089358;
          obj[new_obj_idx].P_psi[1] = 0.0;
          obj[new_obj_idx].P_psi[2] = 0.0;
          obj[new_obj_idx].P_psi[3] = 1.0;
          obj[new_obj_idx].P_obj_type[0] = 0.5;
          obj[new_obj_idx].P_obj_type[1] = 0.5;

          //  IMM-Models initialization
          //  #1 constant turn rate
          obj[new_obj_idx].IMM[0].P[0] = R_final[0];
          obj[new_obj_idx].IMM[0].P[6] = R_final[18];
          obj[new_obj_idx].IMM[0].P[12] = 0.0;
          obj[new_obj_idx].IMM[0].P[18] = 0.0;
          obj[new_obj_idx].IMM[0].P[24] = 0.0;
          obj[new_obj_idx].IMM[0].P[30] = 0.0;
          obj[new_obj_idx].IMM[0].P[1] = R_final[3];
          obj[new_obj_idx].IMM[0].P[7] = R_final[21];
          obj[new_obj_idx].IMM[0].P[13] = 0.0;
          obj[new_obj_idx].IMM[0].P[19] = 0.0;
          obj[new_obj_idx].IMM[0].P[25] = 0.0;
          obj[new_obj_idx].IMM[0].P[31] = 0.0;
          obj[new_obj_idx].IMM[0].x[0] = dx;
          obj[new_obj_idx].IMM[0].x[1] = dy;
          obj[new_obj_idx].IMM[0].x[2] = psi;
          obj[new_obj_idx].IMM[0].x[3] = 0.0;
          obj[new_obj_idx].IMM[0].x[4] = vr;
          obj[new_obj_idx].IMM[0].x[5] = 0.0;
          obj[new_obj_idx].IMM[0].mu = 0.33333333333333331;

          //  #2 crossing high-dyn
          for (i1 = 0; i1 < 6; i1++) {
            obj[new_obj_idx].IMM[0].P[6 * i1 + 2] = dv[i1];
            obj[new_obj_idx].IMM[0].P[6 * i1 + 3] = iv2[i1];
            obj[new_obj_idx].IMM[0].P[6 * i1 + 4] = iv3[i1];
            obj[new_obj_idx].IMM[0].P[6 * i1 + 5] = 0.0;
            obj[new_obj_idx].IMM[1].x[i1] = obj[new_obj_idx].x[i1];
          }

          std::memcpy(&obj[new_obj_idx].IMM[1].P[0], &obj[new_obj_idx].P[0], 36U
                      * sizeof(double));
          obj[new_obj_idx].IMM[1].mu = 0.33333333333333331;

          //  #3 crossing low-dyn
          for (i1 = 0; i1 < 6; i1++) {
            obj[new_obj_idx].IMM[2].x[i1] = obj[new_obj_idx].x[i1];
          }

          std::memcpy(&obj[new_obj_idx].IMM[2].P[0], &obj[new_obj_idx].P[0], 36U
                      * sizeof(double));
          obj[new_obj_idx].IMM[2].mu = 0.33333333333333331;

          //  combine IMM results
          imm_output(&obj[new_obj_idx]);

          //  init local gridmap
          for (i1 = 0; i1 < 441; i1++) {
            obj[new_obj_idx].grid.LR[i1] = 0.1;
          }

          obj[new_obj_idx].grid.LR[(static_cast<int>(std::ceil(obj[new_obj_idx].
            grid.x_len / 2.0)) + 21 * (static_cast<int>(std::ceil
            (obj[new_obj_idx].grid.y_len / 2.0)) - 1)) - 1] = 1.0;

          //  RCS initialize
          obj[new_obj_idx].RCS_filt = -128.0;
        }
      }
    }
  }
}

//
// defs
// Arguments    : const double measurements_asso[512]
//                const double measurements_potDoubleRefl[512]
//                const double measurements_meas[512]
//                const double measurements_dr[512]
//                const double measurements_vr[512]
//                const double measurements_dBRcs[512]
//                double measurements_dx_sens_offset
//                double measurements_dy_sens_offset
//                const double measurements_alpha[512]
//                double vx_ego
//                double dt
//                b_struct_T new_track_candidates[100]
//                double *num_new_tracks
// Return Type  : void
//
static void trackletManagementInit(const double measurements_asso[512], const
  double measurements_potDoubleRefl[512], const double measurements_meas[512],
  const double measurements_dr[512], const double measurements_vr[512], const
  double measurements_dBRcs[512], double measurements_dx_sens_offset, double
  measurements_dy_sens_offset, const double measurements_alpha[512], double
  vx_ego, double dt, b_struct_T new_track_candidates[100], double
  *num_new_tracks)
{
  static b_struct_T b_r = { { 0.0, 0.0, 0.0, 0.0, 0.0 },// r
    { 0.0, 0.0, 0.0, 0.0, 0.0 },       // v
    { 0.0, 0.0, 0.0, 0.0, 0.0 },       // azimuth_global
    { 0.0, 0.0, 0.0, 0.0, 0.0 },       // dt
    0.0,                               // r0
    0.0,                               // v0
    0.0,                               // alpha0
    0.0,                               // psi0
    0.0,                               // var_r0
    0.0,                               // var_v0
    0.0                                // var_alpha0
  };

  int i;
  int trueCount;
  int partialTrueCount;
  int b_i;
  int nx;
  double non_associated_meas_r_data[512];
  short tmp_data[512];
  bool non_associated_meas_inds[512];
  double non_associated_meas_ang_data[512];
  short b_tmp_data[512];
  double a_data[512];
  double c_non_associated_meas_r_global_[512];
  coder::array<c_struct_T, 1U> non_associated_locations_list;
  coder::array<bool, 1U> non_associated_observed;
  short c_tmp_data[512];
  double u1;
  int exitg3;
  double curr_alpha;
  coder::array<c_struct_T, 1U> new_unobserved_locations_list;
  double non_associated_meas_alpha;
  b_struct_T expl_temp;
  b_r.var_alpha0 = rtGetInf();
  b_r.var_v0 = rtGetInf();
  b_r.var_r0 = rtGetInf();

  //  init output
  for (i = 0; i < 100; i++) {
    new_track_candidates[i] = b_r;
  }

  *num_new_tracks = 0.0;

  //  find all non-associated measurements
  trueCount = 0;
  partialTrueCount = 0;
  for (i = 0; i < 512; i++) {
    bool b;
    bool b1;
    b = ((measurements_asso[i] == 0.0) && (measurements_meas[i] == 1.0) &&
         (measurements_potDoubleRefl[i] == 0.0));
    b1 = b;
    non_associated_meas_inds[i] = b;
    if ((measurements_dr[i] < 20.0) && (measurements_dBRcs[i] < -10.0)) {
      b = false;
      b1 = false;
      non_associated_meas_inds[i] = false;
    }

    if (b) {
      trueCount++;
    }

    if (b1) {
      tmp_data[partialTrueCount] = static_cast<short>(i + 1);
      partialTrueCount++;
    }
  }

  for (b_i = 0; b_i < trueCount; b_i++) {
    non_associated_meas_r_data[b_i] = measurements_dr[tmp_data[b_i] - 1];
  }

  nx = 0;
  partialTrueCount = 0;
  for (i = 0; i < 512; i++) {
    if (non_associated_meas_inds[i]) {
      nx++;
      b_tmp_data[partialTrueCount] = static_cast<short>(i + 1);
      partialTrueCount++;
    }
  }

  for (b_i = 0; b_i < nx; b_i++) {
    non_associated_meas_ang_data[b_i] = measurements_alpha[b_tmp_data[b_i] - 1];
  }

  if (0 <= nx - 1) {
    std::memcpy(&a_data[0], &non_associated_meas_ang_data[0], nx * sizeof(double));
  }

  for (partialTrueCount = 0; partialTrueCount < nx; partialTrueCount++) {
    a_data[partialTrueCount] = std::cos(a_data[partialTrueCount]);
  }

  for (partialTrueCount = 0; partialTrueCount < nx; partialTrueCount++) {
    non_associated_meas_ang_data[partialTrueCount] = std::sin
      (non_associated_meas_ang_data[partialTrueCount]);
  }

  nx = trueCount - 1;
  for (b_i = 0; b_i <= nx; b_i++) {
    a_data[b_i] = non_associated_meas_r_data[b_i] * a_data[b_i] +
      measurements_dx_sens_offset;
  }

  i = static_cast<short>(trueCount);
  nx = static_cast<short>(trueCount);
  for (partialTrueCount = 0; partialTrueCount < nx; partialTrueCount++) {
    c_non_associated_meas_r_global_[partialTrueCount] = rt_powd_snf
      (a_data[partialTrueCount], 2.0);
  }

  nx = trueCount - 1;
  for (b_i = 0; b_i <= nx; b_i++) {
    non_associated_meas_r_data[b_i] = non_associated_meas_r_data[b_i] *
      non_associated_meas_ang_data[b_i] + measurements_dy_sens_offset;
  }

  nx = static_cast<short>(trueCount);
  for (partialTrueCount = 0; partialTrueCount < nx; partialTrueCount++) {
    non_associated_meas_ang_data[partialTrueCount] = rt_powd_snf
      (non_associated_meas_r_data[partialTrueCount], 2.0);
  }

  nx = i - 1;
  for (b_i = 0; b_i <= nx; b_i++) {
    c_non_associated_meas_r_global_[b_i] += non_associated_meas_ang_data[b_i];
  }

  for (partialTrueCount = 0; partialTrueCount < i; partialTrueCount++) {
    c_non_associated_meas_r_global_[partialTrueCount] = std::sqrt
      (c_non_associated_meas_r_global_[partialTrueCount]);
  }

  nx = non_associated_meas_inds[0];
  for (partialTrueCount = 0; partialTrueCount < 511; partialTrueCount++) {
    nx += non_associated_meas_inds[partialTrueCount + 1];
  }

  non_associated_locations_list.set_size(nx);
  for (b_i = 0; b_i < nx; b_i++) {
    non_associated_locations_list[b_i].r = 0.0;
    non_associated_locations_list[b_i].v = 0.0;
    non_associated_locations_list[b_i].azimuth_global = 0.0;
    non_associated_locations_list[b_i].range_gate = 0.0;
    non_associated_locations_list[b_i].vel_gate = 0.0;
    non_associated_locations_list[b_i].ang_gate = 0.0;
  }

  b_i = non_associated_locations_list.size(0);
  for (nx = 0; nx < b_i; nx++) {
    non_associated_locations_list[nx].r = c_non_associated_meas_r_global_[nx];
    partialTrueCount = 0;
    for (i = 0; i < 512; i++) {
      if (non_associated_meas_inds[i]) {
        c_tmp_data[partialTrueCount] = static_cast<short>(i + 1);
        partialTrueCount++;
      }
    }

    non_associated_locations_list[nx].v = measurements_vr[c_tmp_data[nx] - 1];
    non_associated_locations_list[nx].azimuth_global =
      measurements_alpha[b_tmp_data[nx] - 1];
    u1 = c_non_associated_meas_r_global_[nx] / 10.0;
    if ((2.0 > u1) || rtIsNaN(u1)) {
      curr_alpha = 2.0;
    } else {
      curr_alpha = u1;
    }

    non_associated_locations_list[nx].range_gate = curr_alpha;
    u1 = c_non_associated_meas_r_global_[nx] / 20.0;
    if ((3.0 > u1) || rtIsNaN(u1)) {
      non_associated_locations_list[nx].vel_gate = 3.0;
    } else {
      non_associated_locations_list[nx].vel_gate = u1;
    }

    non_associated_locations_list[nx].ang_gate = curr_alpha / 180.0 *
      3.1415926535897931;
  }

  //  associate measurements to locations_store
  non_associated_observed.set_size(non_associated_locations_list.size(0));
  nx = non_associated_locations_list.size(0);
  for (b_i = 0; b_i < nx; b_i++) {
    non_associated_observed[b_i] = false;
  }

  partialTrueCount = 0;
  do {
    exitg3 = 0;
    if (partialTrueCount < 1000) {
      if (locations_store[partialTrueCount].last_update_age == rtInf) {
        partialTrueCount++;
      } else {
        double curr_vel;
        double curr_range;
        nx = static_cast<int>(locations_store[partialTrueCount].
                              num_locs_observed) - 1;
        curr_alpha = locations_store[partialTrueCount].azimuth_global[nx];
        curr_vel = locations_store[partialTrueCount].v[nx];

        //  ego-motion compensated radial velocity
        locations_store[partialTrueCount].last_update_age += dt;
        curr_range = locations_store[partialTrueCount].r[static_cast<int>
          (locations_store[partialTrueCount].num_locs_observed) - 1] +
          locations_store[partialTrueCount].v[nx] *
          locations_store[partialTrueCount].last_update_age;

        //  update
        if (locations_store[partialTrueCount].last_update_age > 0.15) {  //lnl, tracker_initial, delta t  //2
          //  2s unobserved
          //  delete this tracklet
          locations_store[partialTrueCount].num_locs_observed = 0.0;
          locations_store[partialTrueCount].last_update_age = rtInf;
          for (i = 0; i < 5; i++) {
            locations_store[partialTrueCount].r[i] = 0.0;
            locations_store[partialTrueCount].v[i] = 0.0;
            locations_store[partialTrueCount].azimuth_global[i] = 0.0;
            locations_store[partialTrueCount].dt[i] = 0.0;
          }

          partialTrueCount++;
        } else {
          int exitg2;
          nx = 0;
          do {
            exitg2 = 0;
            if (nx <= non_associated_locations_list.size(0) - 1) {
              if (locations_store[partialTrueCount].last_update_age != rtInf) {
                double non_associated_meas_v;
                non_associated_meas_v = non_associated_locations_list[nx].v -
                  vx_ego * -std::cos(non_associated_locations_list[nx].
                                     azimuth_global);

                //  substitute for matlab's wrapToPi, which is not supported for code 
                //  generation
                for (non_associated_meas_alpha =
                     non_associated_locations_list[nx].azimuth_global;
                     non_associated_meas_alpha < -3.1415926535897931;
                     non_associated_meas_alpha += 6.2831853071795862) {
                }

                while (non_associated_meas_alpha > 3.1415926535897931) {
                  non_associated_meas_alpha -= 6.2831853071795862;
                }

                if ((std::abs(non_associated_locations_list[nx].r - curr_range) <
                     non_associated_locations_list[nx].range_gate) && (std::abs
                     (non_associated_meas_v - curr_vel) <
                     non_associated_locations_list[nx].vel_gate)) {
                  //  substitute for matlab's wrapToPi, which is not supported for code 
                  //  generation
                  for (u1 = non_associated_meas_alpha - curr_alpha; u1 <
                       -3.1415926535897931; u1 += 6.2831853071795862) {
                  }

                  while (u1 > 3.1415926535897931) {
                    u1 -= 6.2831853071795862;
                  }

                  if (std::abs(u1) < non_associated_locations_list[nx].ang_gate)
                  {
                    //  update this combination!
                    locations_store[partialTrueCount].num_locs_observed++;
                    locations_store[partialTrueCount].dt[static_cast<int>
                      (locations_store[partialTrueCount].num_locs_observed) - 1]
                      = locations_store[partialTrueCount].last_update_age;
                    locations_store[partialTrueCount].last_update_age = 0.0;
                    non_associated_observed[nx] = true;

                    //  mark as associated
                    locations_store[partialTrueCount].r[static_cast<int>
                      (locations_store[partialTrueCount].num_locs_observed) - 1]
                      = non_associated_locations_list[nx].r;
                    locations_store[partialTrueCount].v[static_cast<int>
                      (locations_store[partialTrueCount].num_locs_observed) - 1]
                      = non_associated_meas_v;
                    locations_store[partialTrueCount].azimuth_global[
                      static_cast<int>(locations_store[partialTrueCount].
                                       num_locs_observed) - 1] =
                      non_associated_meas_alpha;

                    //  enough consistent locations observed?
                    if (locations_store[partialTrueCount].num_locs_observed ==
                        5.0) {
                      //  add to output
                      (*num_new_tracks)++;
                      for (i = 0; i < 5; i++) {
                        expl_temp.r[i] = locations_store[partialTrueCount].r[i];
                        expl_temp.v[i] = locations_store[partialTrueCount].v[i];
                        expl_temp.azimuth_global[i] =
                          locations_store[partialTrueCount].azimuth_global[i];
                        expl_temp.dt[i] = locations_store[partialTrueCount].dt[i];
                      #if tracker_initial
                      std::cout<<" 77_mgmt_init, dt: "<<expl_temp.dt[i] <<std::endl;
                      std::cout<<" 77_2_mgmt_init, alpha0: "<<expl_temp.alpha0<<std::endl;
                      std::cout<<" 77_3_mgmt_init, psi0: "<<expl_temp.psi0<<std::endl;

                      #endif


                      }
                      #if tracker_initial
                      std::cout<<" 77_mgmt_init, num_locs_observed: "<<locations_store[partialTrueCount].num_locs_observed<<" new_tracks_num: "<<*num_new_tracks<<std::endl;
                      #endif
                      expl_temp.r0 = 0.0;
                      expl_temp.v0 = 0.0;
                      expl_temp.alpha0 = 0.0;
                      expl_temp.psi0 = 0.0; 
                      expl_temp.var_r0 = rtInf;
                      expl_temp.var_v0 = rtInf;
                      expl_temp.var_alpha0 = rtInf;
                      new_track_candidates[static_cast<int>(*num_new_tracks) - 1]
                        = expl_temp;

                      //  delete from locations_store
                      locations_store[partialTrueCount].num_locs_observed = 0.0;
                      locations_store[partialTrueCount].last_update_age = rtInf;
                      for (i = 0; i < 5; i++) {
                        locations_store[partialTrueCount].r[i] = 0.0;
                        locations_store[partialTrueCount].v[i] = 0.0;
                        locations_store[partialTrueCount].azimuth_global[i] =
                          0.0;
                        locations_store[partialTrueCount].dt[i] = 0.0;
                      }

                      if (*num_new_tracks == 100.0) {
                        exitg2 = 2;
                      } else {
                        nx++;
                      }
                    } else {
                      nx++;
                    }
                  } else {
                    nx++;
                  }
                } else {
                  nx++;
                }
              } else {
                nx++;
              }
            } else {
              exitg2 = 1;
            }
          } while (exitg2 == 0);

          if (exitg2 == 1) {
            partialTrueCount++;
          } else {
            exitg3 = 1;
          }
        }
      }
    } else {
      //  create new entries for non-associated measurements
      nx = non_associated_observed.size(0) - 1;
      trueCount = 0;
      for (i = 0; i <= nx; i++) {
        if (!non_associated_observed[i]) {
          trueCount++;
        }
      }

      new_unobserved_locations_list.set_size(trueCount);
      partialTrueCount = 0;
      for (i = 0; i <= nx; i++) {
        if (!non_associated_observed[i]) {
          new_unobserved_locations_list[partialTrueCount] =
            non_associated_locations_list[i];
          partialTrueCount++;
        }
      }

      exitg3 = 2;
    }
  } while (exitg3 == 0);

  if ((exitg3 != 1) && (new_unobserved_locations_list.size(0) != 0)) {
    unsigned int meas_ind;
    bool exitg1;
    meas_ind = 1U;
    partialTrueCount = 0;
    exitg1 = false;
    while ((!exitg1) && (partialTrueCount < 1000)) {
      if (locations_store[partialTrueCount].num_locs_observed == 0.0) {
        b_i = static_cast<int>(meas_ind) - 1;
        locations_store[partialTrueCount].r[0] =
          new_unobserved_locations_list[b_i].r;
        locations_store[partialTrueCount].v[0] =
          new_unobserved_locations_list[b_i].v - vx_ego * -std::cos
          (new_unobserved_locations_list[b_i].azimuth_global);

        //  substitute for matlab's wrapToPi, which is not supported for code
        //  generation
        for (u1 = new_unobserved_locations_list[b_i].azimuth_global; u1 <
             -3.1415926535897931; u1 += 6.2831853071795862) {
        }

        while (u1 > 3.1415926535897931) {
          u1 -= 6.2831853071795862;
        }

        locations_store[partialTrueCount].azimuth_global[0] = u1;
        locations_store[partialTrueCount].num_locs_observed = 1.0;
        locations_store[partialTrueCount].last_update_age = 0.0;
        meas_ind++;
        if (meas_ind > static_cast<unsigned int>
            (new_unobserved_locations_list.size(0))) {
          exitg1 = true;
        } else {
          partialTrueCount++;
        }
      } else {
        partialTrueCount++;
      }
    }
  }
}

//
// Track initialization (spawning) module
// Arguments    : OBJECT_STRUCT obj[50]
//                const double measurements_asso[512]
//                const double measurements_potDoubleRefl[512]
//                const double measurements_meas[512]
//                const double measurements_dr[512]
//                const double measurements_vr[512]
//                const double measurements_dBRcs[512]
//                double measurements_t
//                double measurements_vx_ego
//                double measurements_dx_sens_offset
//                double measurements_dy_sens_offset
//                const double measurements_alpha[512]
//                double dt
// Return Type  : void
//
void mgmt_init_track(OBJECT_STRUCT obj[50], const double measurements_asso[512],
                     const double measurements_potDoubleRefl[512], const double
                     measurements_meas[512], const double measurements_dr[512],
                     const double measurements_vr[512], const double
                     measurements_dBRcs[512], double measurements_t, double
                     measurements_vx_ego, double measurements_dx_sens_offset,
                     double measurements_dy_sens_offset, const double
                     measurements_alpha[512], double dt)
{
  b_struct_T new_track_candidates[100];
  double num_new_tracks;
  bool bv[100];

  //  store all unassigned measurements
  trackletManagementInit(measurements_asso, measurements_potDoubleRefl,
    measurements_meas, measurements_dr, measurements_vr, measurements_dBRcs,
    measurements_dx_sens_offset, measurements_dy_sens_offset, measurements_alpha,
    measurements_vx_ego, dt, new_track_candidates, &num_new_tracks);

  //  are there any new candidates?
  if (num_new_tracks > 0.0) {
    //  range/angle/velocity regression or track
    new_track_regression(new_track_candidates, num_new_tracks);

    //  fuse new candidates, if overlapping
    //  add new candidates to object list
    merge_tracks_init(new_track_candidates, num_new_tracks, bv);
    set_new_track_properties(obj, new_track_candidates, num_new_tracks, bv,
      measurements_vx_ego, measurements_t);
  }
}

//
// defs
// Arguments    : void
// Return Type  : void
//
void trackletManagementInit_init()
{
  static struct_T b_r = { { 0.0, 0.0, 0.0, 0.0, 0.0 },// r
    { 0.0, 0.0, 0.0, 0.0, 0.0 },       // v
    { 0.0, 0.0, 0.0, 0.0, 0.0 },       // azimuth_global
    0.0,                               // num_locs_observed
    0.0,                               // last_update_age
    { 0.0, 0.0, 0.0, 0.0, 0.0 }        // dt
  };

  b_r.last_update_age = rtGetInf();
  for (int i = 0; i < 1000; i++) {
    locations_store[i] = b_r;
  }

  //  list of 1000 location candidates
}

//
// File trailer for mgmt_init_track.cpp
//
// [EOF]
//
