//
// File: radar_tracker_types.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//
#ifndef RADAR_TRACKER_TYPES_H
#define RADAR_TRACKER_TYPES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable : 4251)

#endif

// Type Definitions
struct struct2_T
{
  double x[6];
  double P[36];
  double mu;
  double pdf_max;
};

struct struct0_T
{
  double step;
  double x_cells[21];
  double y_cells[21];
  double x_len;
  double y_len;
  double LR[441];
};

struct struct1_T
{
  double range_interval;
  double vel_interval;
  double ang_interval;
};

struct OBJECT_STRUCT
{
  bool valid;
  bool meas;
  double meas_angles;
  bool measureable;
  bool hist;
  double age;
  double t_abs;
  double t_lastmeas;
  double standing;
  double moving;
  double pexist;
  double Pgr_reflex;
  double x[6];
  double P[36];
  double length;
  double width;
  double psiDt;
  double psi;
  double P_psi[4];
  double RCS_filt;
  double P_obj_type[2];
  double length_meas;
  double length_filt;
  double width_meas;
  double width_filt;
  double traj_hist_x[20];
  double traj_hist_y[20];
  double traj_hist_vx[20];
  double traj_hist_vy[20];
  double traj_hist_t[20];
  double traj_hist_len;
  double psi_traj_hist;
  struct0_T grid;
  struct1_T asso;
  struct2_T IMM[3];
  double test_delta; //lnl
};

struct e_struct_T
{
  double loc_nr;
  double y[3];
  double R[9];
  double PDH1;
  double PDH0;
  double potGhost;
  double S[27];
  double d2[3];
};

struct MEASLIST_STRUCT
{
  double asso[512];
  double preasso[512];
  double potGhost[512];
  double potDoubleRefl[512];
  double meas[512];
  double dr[512];
  double drVar[512];
  double vr[512];
  double vrVar[512];
  double PDH1[512];
  double PDH0[512];
  double dBRcs[512];
  double t;
  double vx_ego;
  double ax_ego;
  double psiDt_ego;
  double kapCurvTraj;
  double beta_ego;
  double dx_sens_offset;
  double dy_sens_offset;
  double ang_sens_offset;
  double fov_range[25];
  double fov_angle[25];
  double alpha[512];
  double alpVar[512];
};

#ifdef _MSC_VER

#pragma warning(pop)

#endif
#endif

//
// File trailer for radar_tracker_types.h
//
// [EOF]
//
