//
// File: update_derived_values.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "update_derived_values.h"
#include "mgmt_init_track.h"
#include "polyfit.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

//lnl, brake to stop
#include <iostream>

// Function Definitions

//
// update additional state attributes (State UPdate)
//  - orientation and yaw rate
//  - object extent (width/length)
//  - classificationof moving object
// Arguments    : OBJECT_STRUCT *obj
//                const double measurements_dBRcs[512]
//                const e_struct_T asso_list[20]
// Return Type  : void
//
void update_derived_values(OBJECT_STRUCT *obj, const double measurements_dBRcs
  [512], const e_struct_T asso_list[20])
{
  double psi0;
  double buffer_x0[11];
  double buffer_y0[11];
  double alpha;
  int i;
  double max_rcs;
  double prob_grid[441];
  double buffer_x0_data[5];
  double b_buffer_x0_data[5];
  double coef[3];
  double prob_grid_x[21];

  //  calculate object moving direction from historical trajectory --------------------------- 
  if (obj->traj_hist_len > 5.0) {
    //  middle of rear axis
    //  transform object to middle of rear axis
    //  + obj.x_refl(1);
    //  + obj.y_refl(1);
    //  get last position for rough orientation
    //  rough estimate of orientation
    psi0 = rt_atan2d_snf(obj->traj_hist_y[0] - obj->traj_hist_y[1],
                         obj->traj_hist_x[0] - obj->traj_hist_x[1]);

    //  add current position to trajectory-buffer
    std::memset(&buffer_x0[0], 0, 11U * sizeof(double));
    std::memset(&buffer_y0[0], 0, 11U * sizeof(double));
    buffer_x0[0] = obj->traj_hist_x[0];
    buffer_y0[0] = obj->traj_hist_y[0];
    buffer_x0[1] = obj->traj_hist_x[1];
    buffer_y0[1] = obj->traj_hist_y[1];
    buffer_x0[2] = obj->traj_hist_x[2];
    buffer_y0[2] = obj->traj_hist_y[2];
    buffer_x0[3] = obj->traj_hist_x[3];
    buffer_y0[3] = obj->traj_hist_y[3];

    //  rotate to rough orientation, to avoid pole at 90 degree
    max_rcs = std::sin(psi0);
    alpha = std::cos(psi0);

    //  parabola-fit
    for (i = 0; i < 5; i++) {
      buffer_x0_data[i] = buffer_x0[i] * alpha + buffer_y0[i] * max_rcs;
      b_buffer_x0_data[i] = -buffer_x0[i] * max_rcs + buffer_y0[i] * alpha;
    }

    double psi_tmp;
    polyfit(buffer_x0_data, b_buffer_x0_data, coef);

    //  get slope at current position
    //  calculate orientation
    psi_tmp = -0.5 * obj->length;
    psi0 += std::atan(2.0 * coef[0] * ((obj->x[0] + psi_tmp * std::cos
      (obj->psi_traj_hist) * 0.0) * alpha + (obj->x[3] + psi_tmp * std::sin
      (obj->psi_traj_hist) * 0.0) * max_rcs) + coef[1]);

    //  limit to +/- pi
    if (psi0 > 3.1415926535897931) {
      psi0 -= 6.2831853071795862;
    } else {
      if (psi0 < -3.1415926535897931) {
        psi0 += 6.2831853071795862;
      }
    }

    obj->psi_traj_hist = psi0;
  }

  //  output orientation
  if (std::sqrt(obj->x[1] * obj->x[1] + obj->x[4] * obj->x[4]) < 1.0) {
    if (obj->traj_hist_len > 5.0) {
      obj->psi = obj->psi_traj_hist;
    } else {
      // obj.psi = obj.IMM(1).x(3);          % use the CTR state
      obj->psi = rt_atan2d_snf(obj->x[4], obj->x[1]);

      //  use the object velocity
    }
  } else {
    // obj.psi = obj.IMM(1).x(3);         % use the CTR state
    obj->psi = rt_atan2d_snf(obj->x[4], obj->x[1]);

    //  use the object velocity
  }

  //  extent (length/width) estimation ------------------------------------------------------------- 
  if (obj->meas) {
    int lr_y_end;
    int idx;

    //  new information only through measurements (moving classification)
    alpha = rt_atan2d_snf(obj->x[3], obj->x[0]);

    //  radial velovity over ground
    if ((obj->moving == 1.0) && ((obj->traj_hist_len >= 5.0) || (std::abs(std::
           cos(alpha) * obj->x[1] + std::sin(alpha) * obj->x[4]) > 2.0))) {
      int k;
      bool exitg1;
      int lr_x_begin;
      int lr_x_end;

      //  orientation reliable?
      for (i = 0; i < 441; i++) {
        prob_grid[i] = obj->grid.LR[i] / (obj->grid.LR[i] + 1.0);
      }

      //  convert odds to prob
      if (!rtIsNaN(prob_grid[0])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= 441)) {
          if (!rtIsNaN(prob_grid[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        psi0 = prob_grid[0];
      } else {
        psi0 = prob_grid[idx - 1];
        i = idx + 1;
        for (k = i; k < 442; k++) {
          max_rcs = prob_grid[k - 1];
          if (psi0 > max_rcs) {
            psi0 = max_rcs;
          }
        }
      }

      for (i = 0; i < 441; i++) {
        prob_grid[i] -= psi0;
      }

      std::memcpy(&prob_grid_x[0], &prob_grid[0], 21U * sizeof(double));
      for (k = 0; k < 20; k++) {
        idx = (k + 1) * 21;
        for (lr_y_end = 0; lr_y_end < 21; lr_y_end++) {
          prob_grid_x[lr_y_end] += prob_grid[idx + lr_y_end];
        }
      }

      for (k = 0; k < 20; k++) {
        prob_grid_x[k + 1] += prob_grid_x[k];
      }

      if (!rtIsNaN(prob_grid_x[0])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= 21)) {
          if (!rtIsNaN(prob_grid_x[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        psi0 = prob_grid_x[0];
      } else {
        psi0 = prob_grid_x[idx - 1];
        i = idx + 1;
        for (k = i; k < 22; k++) {
          max_rcs = prob_grid_x[k - 1];
          if (psi0 < max_rcs) {
            psi0 = max_rcs;
          }
        }
      }

      for (i = 0; i < 21; i++) {
        prob_grid_x[i] /= psi0;
      }

      lr_x_begin = -1;
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 21)) {
        if (prob_grid_x[i] > 0.3) {
          lr_x_begin = i;
          exitg1 = true;
        } else {
          i++;
        }
      }

      lr_x_end = -1;
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 21)) {
        if (prob_grid_x[i] > 0.7) {
          lr_x_end = i;
          exitg1 = true;
        } else {
          i++;
        }
      }

      for (i = 0; i < 21; i++) {
        idx = i * 21;
        max_rcs = prob_grid[idx];
        for (k = 0; k < 20; k++) {
          max_rcs += prob_grid[(idx + k) + 1];
        }

        prob_grid_x[i] = max_rcs;
      }

      for (k = 0; k < 20; k++) {
        prob_grid_x[k + 1] += prob_grid_x[k];
      }

      if (!rtIsNaN(prob_grid_x[0])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= 21)) {
          if (!rtIsNaN(prob_grid_x[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        psi0 = prob_grid_x[0];
      } else {
        psi0 = prob_grid_x[idx - 1];
        i = idx + 1;
        for (k = i; k < 22; k++) {
          max_rcs = prob_grid_x[k - 1];
          if (psi0 < max_rcs) {
            psi0 = max_rcs;
          }
        }
      }

      for (i = 0; i < 21; i++) {
        prob_grid_x[i] /= psi0;
      }

      idx = -1;
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 21)) {
        if (prob_grid_x[i] > 0.3) {
          idx = i;
          exitg1 = true;
        } else {
          i++;
        }
      }

      lr_y_end = -1;
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 21)) {
        if (prob_grid_x[i] > 0.7) {
          lr_y_end = i;
          exitg1 = true;
        } else {
          i++;
        }
      }

      obj->length_meas = static_cast<double>(lr_x_end - lr_x_begin) +
        obj->grid.step;
      obj->width_meas = static_cast<double>(lr_y_end - idx) + obj->grid.step;
      obj->length_filt = 0.9 * obj->length_filt + 0.1 * obj->length_meas;
      obj->width_filt = 0.9 * obj->width_filt + 0.1 * obj->width_meas;
      psi0 = obj->length_filt;
      if (!(psi0 > 1.0)) {
        psi0 = 1.0;
      }

      obj->length = psi0;
      //lnl, test
      obj->length = std::min(psi0,(double)10);

      psi0 = obj->width_filt;
      if (!(psi0 > 1.0)) {
        psi0 = 1.0;
      }

      // obj->width = psi0;
      //lnl,test
      obj->width = std::min(psi0,(double)2.5);

    }

    //  filter RCS ------------------------------------------------------------------ 
    //  search location with strongest RCS
    max_rcs = -128.0;
    for (lr_y_end = 0; lr_y_end < 20; lr_y_end++) {
      if (asso_list[lr_y_end].loc_nr > 0.0) {
        idx = 0;
        psi0 = 0.0;
        i = static_cast<int>(asso_list[lr_y_end].loc_nr) - 1;
        if (!(measurements_dBRcs[i] < -127.0)) {
          psi0 = measurements_dBRcs[i];
          idx = 1;
        }

        psi0 /= static_cast<double>(idx);
        if ((!(max_rcs > psi0)) && (!rtIsNaN(psi0))) {
          max_rcs = psi0;
        }
      }
    }

    //  RCS low-pass filter
    if (max_rcs > -128.0) {
      obj->RCS_filt = 0.9 * obj->RCS_filt + 0.1 * max_rcs;
    }

    //  Moving classification ---------------------------------------------------------- 
    psi0 = std::sin(alpha);

    //  radial velocity
    //  velocity over ground
    max_rcs = std::abs(psi0);
    psi0 = max_rcs * std::abs(std::cos(alpha) * obj->x[1] + psi0 * obj->x[4]) +
      (1.0 - max_rcs) * std::sqrt(obj->x[1] * obj->x[1] + obj->x[4] * obj->x[4]);


    //lnl, brake to stop
    // max_rcs = 0.1 * (1.0 - obj->moving) + 0.95 * obj->moving;
    // auto test = max_rcs * 0.1 / (max_rcs * 0.1 + (0.9 * (1.0 - obj->moving)
    //     + 0.05 * obj->moving) * (0.79788456080286541 * std::exp(-0.5 * (psi0 *
    //     psi0) / 0.25)));
    // std::cout<<"000_test: "<<test<<std::endl;

    //  weighting: front=v_ground, side=v_radial
    if ((obj->moving < 0.99) || (obj->age < 1.0)) {
      //  "lock" at high velocity
      //  transition probability
      //  old hypothesis 1: not mobile
      //  old hypothesis 2: mobile
      //  new not mobile | new mobile
      //  calculation of a-priori hypothesis probability
      max_rcs = 0.1 * (1.0 - obj->moving) + 0.95 * obj->moving;

      //  calculation of current hypothesis probabilities
      //  uniform distribution  1/(10 m/s)
      //  calculation of the a-posteriori hypothesis probabilities
      obj->moving = max_rcs * 0.1 / (max_rcs * 0.1 + (0.9 * (1.0 - obj->moving)
        + 0.05 * obj->moving) * (0.79788456080286541 * std::exp(-0.5 * (psi0 *
        psi0) / 0.25)));
    } else {
      obj->moving = 1.0;

      //  mobile once => always mobile
    }

    //lnl, brake to stop
    // std::cout<<" update_derived_values: "<<" obj->moving: "<<obj->moving<<" max_rcs: "<<max_rcs<<" psi0: "<<psi0<<std::endl;

    obj->standing = 1.0 - obj->moving;
  }
}

//
// File trailer for update_derived_values.cpp
//
// [EOF]
//
