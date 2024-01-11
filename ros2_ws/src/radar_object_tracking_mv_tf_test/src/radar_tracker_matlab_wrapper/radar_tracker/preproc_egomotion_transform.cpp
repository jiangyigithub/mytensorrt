//
// File: preproc_egomotion_transform.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "preproc_egomotion_transform.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// shift the objects by the ego_motion*dt, because tracking is in ego-local
//  CoSy.
// Arguments    : OBJECT_STRUCT *obj
//                double meas_list_vx_ego
//                double meas_list_psiDt_ego
//                double dt
// Return Type  : void
//
void preproc_egomotion_transform(OBJECT_STRUCT *obj, double meas_list_vx_ego,
  double meas_list_psiDt_ego, double dt)
{
  double d;
  double delta_x;
  double delta_phi;
  double delta_y;
  double delta_x_tmp;
  double x_abs[6];
  double b;
  int i;
  double Tsin[9];
  static const signed char a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double Tcos[9];
  double d1;
  double T[36];
  int i1;
  int T_tmp;
  double T2[4];
  double b_T[36];
  double b_T2[2];
  int i2;

  //  transform IMMs
  d = std::abs(meas_list_psiDt_ego);
  if (d > 0.0001) {
    delta_phi = meas_list_psiDt_ego * dt;
    delta_x_tmp = meas_list_vx_ego / meas_list_psiDt_ego;
    delta_x = delta_x_tmp * std::sin(delta_phi);
    delta_y = delta_x_tmp * (1.0 - std::cos(delta_phi));
  } else {
    delta_x = meas_list_vx_ego * dt;
    delta_y = 0.0;
    delta_phi = 0.0;
  }

  for (int imm_idx = 0; imm_idx < 3; imm_idx++) {
    //  TODO: also update vy_ego and ax_ego.
    //  end function
    //  ------------------------------------------------------------------------------------------------------------ 
    //  Perform coordinate transform
    if (imm_idx + 1 == 1) {
      double T2_tmp;

      //  translation
      x_abs[0] = delta_x;
      x_abs[1] = delta_y;
      x_abs[2] = 0.0;
      x_abs[3] = 0.0;
      x_abs[4] = 0.0;
      x_abs[5] = 0.0;
      for (i = 0; i < 6; i++) {
        x_abs[i] = obj->IMM[imm_idx].x[i] - x_abs[i];
      }

      std::memcpy(&T[0], &obj->IMM[imm_idx].P[0], 36U * sizeof(double));

      //  do not change covariance
      //  define rotation matrix
      b = std::sin(delta_phi);
      T2_tmp = std::cos(delta_phi);
      T2[0] = T2_tmp;
      T2[2] = b;
      T2[1] = -b;
      T2[3] = T2_tmp;

      //  perform rotation
      b_T2[0] = T2_tmp * x_abs[0] + b * x_abs[1];
      b_T2[1] = -b * x_abs[0] + T2_tmp * x_abs[1];
      for (i = 0; i < 2; i++) {
        x_abs[i] = b_T2[i];
        d1 = T2[i + 2];
        delta_x_tmp = T2[i] * obj->IMM[imm_idx].P[0] + d1 * obj->IMM[imm_idx].P
          [1];
        d1 = T2[i] * obj->IMM[imm_idx].P[6] + d1 * obj->IMM[imm_idx].P[7];
        T[i] = delta_x_tmp * T2_tmp + d1 * b;
        T[i + 6] = delta_x_tmp * -b + d1 * T2_tmp;
      }

      x_abs[2] -= delta_phi;

      //  psi, heading
      for (i = 0; i < 6; i++) {
        obj->IMM[imm_idx].x[i] = x_abs[i];
      }

      std::memcpy(&obj->IMM[imm_idx].P[0], &T[0], 36U * sizeof(double));
    } else {
      //  Update location (ego-motion compensation)
      //  define rotation matrix
      delta_x_tmp = std::sin(delta_phi);
      b = std::cos(delta_phi);
      for (i = 0; i < 9; i++) {
        Tsin[i] = static_cast<double>(a[i]) * delta_x_tmp;
        Tcos[i] = static_cast<double>(a[i]) * b;
      }

      for (i = 0; i < 3; i++) {
        d1 = Tcos[3 * i];
        T[6 * i] = d1;
        delta_x_tmp = Tsin[3 * i];
        T_tmp = 6 * (i + 3);
        T[T_tmp] = delta_x_tmp;
        T[6 * i + 3] = -delta_x_tmp;
        T[T_tmp + 3] = d1;
        i1 = 3 * i + 1;
        T[6 * i + 1] = Tcos[i1];
        T[T_tmp + 1] = Tsin[i1];
        T[6 * i + 4] = -Tsin[i1];
        T[T_tmp + 4] = Tcos[i1];
        i1 = 3 * i + 2;
        T[6 * i + 2] = Tcos[i1];
        T[T_tmp + 2] = Tsin[i1];
        T[6 * i + 5] = -Tsin[i1];
        T[T_tmp + 5] = Tcos[i1];
      }

      //  perform rotation
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          d1 = 0.0;
          for (T_tmp = 0; T_tmp < 6; T_tmp++) {
            d1 += T[i + 6 * T_tmp] * obj->IMM[imm_idx].P[T_tmp + 6 * i1];
          }

          b_T[i + 6 * i1] = d1;
        }
      }

      x_abs[0] = delta_x;
      x_abs[1] = 0.0;
      x_abs[2] = 0.0;
      x_abs[3] = delta_y;
      x_abs[4] = 0.0;
      x_abs[5] = 0.0;
      for (i = 0; i < 6; i++) {
        for (i1 = 0; i1 < 6; i1++) {
          T_tmp = i + 6 * i1;
          obj->IMM[imm_idx].P[T_tmp] = 0.0;
          for (i2 = 0; i2 < 6; i2++) {
            obj->IMM[imm_idx].P[T_tmp] += b_T[i + 6 * i2] * T[i1 + 6 * i2];
          }
        }

        x_abs[i] = obj->IMM[imm_idx].x[i] - x_abs[i];
      }

      for (i = 0; i < 6; i++) {
        obj->IMM[imm_idx].x[i] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          obj->IMM[imm_idx].x[i] += T[i + 6 * i1] * x_abs[i1];
        }
      }
    }

    //  transform yaw angle
    //  transform historical trajectories
  }

  //  transform fused state
  //  TODO: also update vy_ego and ax_ego.
  //  end function
  //  ------------------------------------------------------------------------------------------------------------ 
  //  Perform coordinate transform
  if (d > 0.0001) {
    delta_phi = meas_list_psiDt_ego * dt;
    delta_x_tmp = meas_list_vx_ego / meas_list_psiDt_ego;
    delta_x = delta_x_tmp * std::sin(delta_phi);
    delta_y = delta_x_tmp * (1.0 - std::cos(delta_phi));
  } else {
    delta_x = meas_list_vx_ego * dt;
    delta_y = 0.0;
    delta_phi = 0.0;
  }

  //  Update location (ego-motion compensation)
  //  define rotation matrix
  delta_x_tmp = std::sin(delta_phi);
  b = std::cos(delta_phi);
  for (i = 0; i < 9; i++) {
    Tsin[i] = static_cast<double>(a[i]) * delta_x_tmp;
    Tcos[i] = static_cast<double>(a[i]) * b;
  }

  for (i = 0; i < 3; i++) {
    d = Tcos[3 * i];
    T[6 * i] = d;
    d1 = Tsin[3 * i];
    T_tmp = 6 * (i + 3);
    T[T_tmp] = d1;
    T[6 * i + 3] = -d1;
    T[T_tmp + 3] = d;
    i1 = 3 * i + 1;
    T[6 * i + 1] = Tcos[i1];
    T[T_tmp + 1] = Tsin[i1];
    T[6 * i + 4] = -Tsin[i1];
    T[T_tmp + 4] = Tcos[i1];
    i1 = 3 * i + 2;
    T[6 * i + 2] = Tcos[i1];
    T[T_tmp + 2] = Tsin[i1];
    T[6 * i + 5] = -Tsin[i1];
    T[T_tmp + 5] = Tcos[i1];
  }

  //  perform rotation
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      d = 0.0;
      for (T_tmp = 0; T_tmp < 6; T_tmp++) {
        d += T[i + 6 * T_tmp] * obj->P[T_tmp + 6 * i1];
      }

      b_T[i + 6 * i1] = d;
    }
  }

  x_abs[0] = obj->x[0] - delta_x;
  x_abs[1] = obj->x[1];
  x_abs[2] = obj->x[2];
  x_abs[3] = obj->x[3] - delta_y;
  x_abs[4] = obj->x[4];
  x_abs[5] = obj->x[5];
  for (i = 0; i < 6; i++) {
    obj->x[i] = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      T_tmp = i + 6 * i1;
      obj->P[T_tmp] = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        obj->P[T_tmp] += b_T[i + 6 * i2] * T[i1 + 6 * i2];
      }

      obj->x[i] += T[T_tmp] * x_abs[i1];
    }
  }

  //  transform yaw angle
  obj->psi -= delta_phi;

  //  limit to +/- pi
  if (obj->psi > 3.1415926535897931) {
    obj->psi -= 6.2831853071795862;
  } else {
    if (obj->psi < -3.1415926535897931) {
      obj->psi += 6.2831853071795862;
    }
  }

  obj->psi_traj_hist -= delta_phi;

  //  limit to +/- pi
  if (obj->psi_traj_hist > 3.1415926535897931) {
    obj->psi_traj_hist -= 6.2831853071795862;
  } else {
    if (obj->psi_traj_hist < -3.1415926535897931) {
      obj->psi_traj_hist += 6.2831853071795862;
    }
  }

  //  transform historical trajectories
  if (obj->traj_hist_len > 0.0) {
    //  translation
    //  rotation
    //  write back historical trajectory
    for (i = 0; i < 20; i++) {
      obj->traj_hist_x[i] -= delta_x;
      obj->traj_hist_y[i] -= delta_y;
      d = -delta_x_tmp * obj->traj_hist_x[i];
      d1 = delta_x_tmp * obj->traj_hist_vy[i];
      obj->traj_hist_vy[i] = -delta_x_tmp * obj->traj_hist_vx[i] + b *
        obj->traj_hist_vy[i];
      obj->traj_hist_x[i] = b * obj->traj_hist_x[i] + delta_x_tmp *
        obj->traj_hist_y[i];
      obj->traj_hist_y[i] = d + b * obj->traj_hist_y[i];
      obj->traj_hist_vx[i] = b * obj->traj_hist_vx[i] + d1;
    }

    //  trajectory remains in ego-local coordinate system
  }
}

//
// File trailer for preproc_egomotion_transform.cpp
//
// [EOF]
//
