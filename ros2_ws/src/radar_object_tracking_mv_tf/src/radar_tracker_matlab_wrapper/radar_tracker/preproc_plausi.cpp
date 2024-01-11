//
// File: preproc_plausi.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "preproc_plausi.h"
#include "mgmt_init_track.h"
#include "mrdivide_helper.h"
#include "qrsolve.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "randsample.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Declarations
static double estimate_vxego_ransac(const double vr_data[], const int vr_size[2],
  const double alpha_data[], const int alpha_size[2]);

// Function Definitions

//
// peform ransac with ransac_num_samples samples
// Arguments    : const double vr_data[]
//                const int vr_size[2]
//                const double alpha_data[]
//                const int alpha_size[2]
// Return Type  : double
//
static double estimate_vxego_ransac(const double vr_data[], const int vr_size[2],
  const double alpha_data[], const int alpha_size[2])
{
  double vx_ego;
  double best_trial_inds[10];
  int best_num_inliers;
  int loop_ub_tmp;
  int nx;
  int loop_ub;
  int y_size_idx_1;
  double curr_trial_inds[10];
  bool y;
  int k;
  bool exitg1;
  double a;
  double vr[10];
  double curr_alpha[10];
  int curr_alpha_tmp;
  double b_data[512];
  int nz;
  bool inlier_inds_data[512];
  double y_data[512];
  short tmp_data[512];
  int y_size[1];
  double b_y_data[512];
  int b_size[1];
  std::memset(&best_trial_inds[0], 0, 10U * sizeof(double));
  best_num_inliers = 0;
  loop_ub_tmp = alpha_size[0] * alpha_size[1];
  nx = alpha_size[1];
  loop_ub = alpha_size[1] - 1;
  y_size_idx_1 = static_cast<short>(alpha_size[1]);
  for (int trial_num = 0; trial_num < 100; trial_num++) {
    randsample(static_cast<double>(vr_size[1]), curr_trial_inds);
    for (k = 0; k < 10; k++) {
      curr_alpha_tmp = static_cast<int>(curr_trial_inds[k]) - 1;
      curr_alpha[k] = std::cos(alpha_data[curr_alpha_tmp]);
      vr[k] = -vr_data[curr_alpha_tmp];
    }

    a = -mrdiv(vr, curr_alpha);
    if (0 <= loop_ub_tmp - 1) {
      std::memcpy(&b_data[0], &alpha_data[0], loop_ub_tmp * sizeof(double));
    }

    for (k = 0; k < nx; k++) {
      b_data[k] = std::cos(b_data[k]);
    }

    for (k = 0; k <= loop_ub; k++) {
      double d;
      d = a * b_data[k] - vr_data[k];
      b_data[k] = d;
      y_data[k] = std::abs(d);
    }

    for (nz = 0; nz < y_size_idx_1; nz++) {
      inlier_inds_data[nz] = (y_data[nz] < 0.1);
    }

    if (y_size_idx_1 == 0) {
      nz = 0;
    } else {
      nz = inlier_inds_data[0];
      for (k = 2; k <= y_size_idx_1; k++) {
        nz += inlier_inds_data[k - 1];
      }
    }

    if (nz > best_num_inliers) {
      best_num_inliers = nz;
      std::memcpy(&best_trial_inds[0], &curr_trial_inds[0], 10U * sizeof(double));
    }
  }

  //  determine all inliers
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 10)) {
    if (!(best_trial_inds[k] != 0.0)) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (y) {
    for (k = 0; k < 10; k++) {
      curr_alpha_tmp = static_cast<int>(best_trial_inds[k]) - 1;
      curr_trial_inds[k] = std::cos(alpha_data[curr_alpha_tmp]);
      vr[k] = -vr_data[curr_alpha_tmp];
    }

    a = -mrdiv(vr, curr_trial_inds);
    curr_alpha_tmp = alpha_size[1];
    if (0 <= loop_ub_tmp - 1) {
      std::memcpy(&b_data[0], &alpha_data[0], loop_ub_tmp * sizeof(double));
    }

    nx = alpha_size[1];
    for (k = 0; k < nx; k++) {
      b_data[k] = std::cos(b_data[k]);
    }

    loop_ub = alpha_size[1] - 1;
    for (nz = 0; nz <= loop_ub; nz++) {
      b_data[nz] = a * b_data[nz] - vr_data[nz];
    }

    y_size_idx_1 = static_cast<short>(alpha_size[1]);
    for (k = 0; k < curr_alpha_tmp; k++) {
      y_data[k] = std::abs(b_data[k]);
    }

    for (nz = 0; nz < y_size_idx_1; nz++) {
      inlier_inds_data[nz] = (y_data[nz] < 0.1);
    }

    //  re-estimate vx_ego using the inliers
    curr_alpha_tmp = y_size_idx_1 - 1;
    nx = 0;
    nz = 0;
    for (best_num_inliers = 0; best_num_inliers <= curr_alpha_tmp;
         best_num_inliers++) {
      if (inlier_inds_data[best_num_inliers]) {
        nx++;
        tmp_data[nz] = static_cast<short>(best_num_inliers + 1);
        nz++;
      }
    }

    for (nz = 0; nz < nx; nz++) {
      b_data[nz] = -vr_data[tmp_data[nz] - 1];
    }

    for (nz = 0; nz < nx; nz++) {
      y_data[nz] = alpha_data[tmp_data[nz] - 1];
    }

    for (k = 0; k < nx; k++) {
      y_data[k] = std::cos(y_data[k]);
    }

    if (nx == 0) {
      vx_ego = 0.0;
    } else if (1 == nx) {
      vx_ego = b_data[0] / y_data[0];
    } else {
      y_size[0] = nx;
      if (0 <= nx - 1) {
        std::memcpy(&b_y_data[0], &y_data[0], nx * sizeof(double));
      }

      b_size[0] = nx;
      if (0 <= nx - 1) {
        std::memcpy(&y_data[0], &b_data[0], nx * sizeof(double));
      }

      vx_ego = qrsolve(b_y_data, y_size, y_data, b_size);
    }
  } else {
    vx_ego = rtInf;
  }

  return vx_ego;
}

//
// Preprocessing of the measurement data
// Arguments    : MEASLIST_STRUCT *meas_list
// Return Type  : void
//
void preproc_plausi(MEASLIST_STRUCT *meas_list)
{
  double v_t_yaw_rate;
  int trueCount;
  double sens_mount_angle;
  int partialTrueCount;
  double vx_ego;
  int i;
  double vy_ego;
  int meas_list_size[2];
  bool location_in_front;
  int b_trueCount;
  short tmp_data[512];
  int b_meas_list_size[2];
  double meas_list_data[512];
  double b_meas_list_data[512];
  double delta_v_standing_abs[3];
  if (std::abs(meas_list->psiDt_ego) < 0.05) {
    //  estimate vx_ego from locations
    trueCount = 0;
    partialTrueCount = 0;
    for (i = 0; i < 512; i++) {
      location_in_front = (meas_list->meas[i] == 1.0);
      if (location_in_front) {
        trueCount++;
        tmp_data[partialTrueCount] = static_cast<short>(i + 1);
        partialTrueCount++;
      }
    }

    meas_list_size[0] = 1;
    meas_list_size[1] = trueCount;
    for (b_trueCount = 0; b_trueCount < trueCount; b_trueCount++) {
      meas_list_data[b_trueCount] = meas_list->vr[tmp_data[b_trueCount] - 1];
    }

    b_meas_list_size[0] = 1;
    b_meas_list_size[1] = trueCount;
    for (b_trueCount = 0; b_trueCount < trueCount; b_trueCount++) {
      b_meas_list_data[b_trueCount] = meas_list->alpha[tmp_data[b_trueCount] - 1];
    }

    vx_ego = estimate_vxego_ransac(meas_list_data, meas_list_size,
      b_meas_list_data, b_meas_list_size);

    //  Avoid, that overwrite with a completely wrong velocity
    v_t_yaw_rate = 0.1 * std::abs(meas_list->vx_ego);
    if ((0.8 > v_t_yaw_rate) || rtIsNaN(v_t_yaw_rate)) {
      v_t_yaw_rate = 0.8;
    }

    if (std::abs(vx_ego - meas_list->vx_ego) < v_t_yaw_rate) {
      meas_list->vx_ego = vx_ego;
    } else {
      //  disp('that did not work')
    }
  }

  v_t_yaw_rate = -meas_list->psiDt_ego * std::sqrt(meas_list->dx_sens_offset *
    meas_list->dx_sens_offset + meas_list->dy_sens_offset *
    meas_list->dy_sens_offset);
  sens_mount_angle = rt_atan2d_snf(meas_list->dy_sens_offset,
    meas_list->dx_sens_offset);
  vx_ego = meas_list->vx_ego + v_t_yaw_rate * std::sin(sens_mount_angle);
  vy_ego = -v_t_yaw_rate * std::cos(sens_mount_angle);

  //  plausibilisation of locations
  for (int idx1_meas = 0; idx1_meas < 511; idx1_meas++) {
    if (meas_list->meas[idx1_meas] == 1.0) {
      bool exitg1;
      double vr_ego_unamb_tmp;

      //  Delete multipath reflections
      if (meas_list->dr[idx1_meas] < 30.0) {
        partialTrueCount = 0;
        exitg1 = false;
        while ((!exitg1) && (partialTrueCount <= 510 - idx1_meas)) {
          i = (idx1_meas + partialTrueCount) + 1;
          if ((meas_list->meas[i] == 1.0) && (meas_list->dr[i] < 30.0)) {
            if (std::abs(meas_list->dr[idx1_meas] - 0.5 * meas_list->dr[i]) <
                1.0) {
              if (std::abs(meas_list->vr[idx1_meas] - 0.5 * meas_list->vr[i]) <
                  0.5) {
                trueCount = 0;
                if ((!rtIsInf(meas_list->alpha[idx1_meas])) && (!rtIsNaN
                     (meas_list->alpha[idx1_meas]))) {
                  trueCount = 1;
                  v_t_yaw_rate = meas_list->alpha[idx1_meas];
                } else {
                  v_t_yaw_rate = 0.0;
                }

                b_trueCount = 0;
                if ((!rtIsInf(meas_list->alpha[i])) && (!rtIsNaN
                     (meas_list->alpha[i]))) {
                  b_trueCount = 1;
                  sens_mount_angle = meas_list->alpha[i];
                } else {
                  sens_mount_angle = 0.0;
                }

                if (std::abs(v_t_yaw_rate / static_cast<double>(trueCount) -
                             sens_mount_angle / static_cast<double>(b_trueCount))
                    < 0.17453292519943295) {
                  meas_list->potDoubleRefl[i] = 1.0;
                  meas_list->meas[i] = 0.0;
                }
              }

              partialTrueCount++;
            } else if ((std::abs(meas_list->dr[i] - 0.5 * meas_list->
                                 dr[idx1_meas]) < 1.0) && (std::abs
                        (meas_list->vr[i] - 0.5 * meas_list->vr[idx1_meas]) <
                        0.5)) {
              trueCount = 0;
              if ((!rtIsInf(meas_list->alpha[idx1_meas])) && (!rtIsNaN
                   (meas_list->alpha[idx1_meas]))) {
                trueCount = 1;
                v_t_yaw_rate = meas_list->alpha[idx1_meas];
              } else {
                v_t_yaw_rate = 0.0;
              }

              b_trueCount = 0;
              if ((!rtIsInf(meas_list->alpha[i])) && (!rtIsNaN(meas_list->
                    alpha[i]))) {
                b_trueCount = 1;
                sens_mount_angle = meas_list->alpha[i];
              } else {
                sens_mount_angle = 0.0;
              }

              if (std::abs(v_t_yaw_rate / static_cast<double>(trueCount) -
                           sens_mount_angle / static_cast<double>(b_trueCount)) <
                  0.17453292519943295) {
                meas_list->potDoubleRefl[idx1_meas] = 1.0;
                meas_list->meas[idx1_meas] = 0.0;
                exitg1 = true;
              } else {
                partialTrueCount++;
              }
            } else {
              partialTrueCount++;
            }
          } else {
            partialTrueCount++;
          }
        }
      }

      //  discard stationary locations
      sens_mount_angle = std::cos(meas_list->alpha[idx1_meas]);
      vr_ego_unamb_tmp = std::sin(meas_list->alpha[idx1_meas]);
      v_t_yaw_rate = 0.1 * std::abs(meas_list->vx_ego);
      if ((0.8 > v_t_yaw_rate) || rtIsNaN(v_t_yaw_rate)) {
        v_t_yaw_rate = 0.8;
      }

      if (std::abs(meas_list->vr[idx1_meas] - (-(vx_ego * sens_mount_angle +
             vy_ego * vr_ego_unamb_tmp))) < v_t_yaw_rate) {
        // 0.8
        meas_list->meas[idx1_meas] = 0.0;

        //  ignore stationary locations
      }

      if (meas_list->vx_ego > 0.1) {
        bool location_behind;
        sens_mount_angle = meas_list->dr[idx1_meas] * sens_mount_angle +
          meas_list->dx_sens_offset;
        v_t_yaw_rate = meas_list->dr[idx1_meas] * vr_ego_unamb_tmp +
          meas_list->dy_sens_offset;

        //  intentionally NOT use atan2; if loc behind us, we want to get
        vr_ego_unamb_tmp = std::abs(v_t_yaw_rate);
        if ((vr_ego_unamb_tmp < 2.0) && (sens_mount_angle > 0.0)) {
          location_in_front = true;
        } else {
          location_in_front = false;
        }

        //  location in front of us (on lane?)
        if ((vr_ego_unamb_tmp < 2.0) && (sens_mount_angle < 0.0)) {
          location_behind = true;
        } else {
          location_behind = false;
        }

        //  location behind us (on lane?)
        if (meas_list->dr[idx1_meas] < 20.0) {
          //  near range
          if (location_in_front) {
            if ((meas_list->vr[idx1_meas] < 0.0) && (meas_list->vr[idx1_meas] >
                 -meas_list->vx_ego * 1.1) && (meas_list->vr[idx1_meas] <
                 -meas_list->vx_ego * 0.3)) {
              //  probably a ground reflex
              meas_list->meas[idx1_meas] = 0.0;
            }
          } else {
            if (location_behind && (meas_list->vr[idx1_meas] > 0.0) &&
                (meas_list->vr[idx1_meas] < meas_list->vx_ego * 1.1) &&
                (meas_list->vr[idx1_meas] > meas_list->vx_ego * 0.3)) {
              //  probably a ground reflex
              meas_list->meas[idx1_meas] = 0.0;
            }
          }
        }

        if ((((!location_in_front) && (!location_behind)) || (meas_list->
              dr[idx1_meas] > 20.0)) && (std::abs(std::atan(v_t_yaw_rate /
               sens_mount_angle)) > 0.17453292519943295)) {
          //  location not on our lane
          //  exclude follower vehicle
          //  Gen5 corner B1.2 sample has 45-degree ambiguities, due to antenna design. 
          //  We try to find the angle, which would fit best for an ego-motion compensation.  
          //  (Assuming target is stationary)
          //  90-degree ambiguity
          v_t_yaw_rate = meas_list->alpha[idx1_meas];
          sens_mount_angle = meas_list->vr[idx1_meas];
          delta_v_standing_abs[0] = std::abs(sens_mount_angle - (-(vx_ego * std::
            cos(v_t_yaw_rate + -1.5707963267948966) + vy_ego * std::sin
            (v_t_yaw_rate + -1.5707963267948966))));
          delta_v_standing_abs[1] = std::abs(sens_mount_angle - (-(vx_ego * std::
            cos(v_t_yaw_rate) + vy_ego * std::sin(v_t_yaw_rate))));
          delta_v_standing_abs[2] = std::abs(sens_mount_angle - (-(vx_ego * std::
            cos(v_t_yaw_rate + 1.5707963267948966) + vy_ego * std::sin
            (v_t_yaw_rate + 1.5707963267948966))));
          if (!rtIsNaN(delta_v_standing_abs[0])) {
            partialTrueCount = 1;
          } else {
            partialTrueCount = 0;
            i = 2;
            exitg1 = false;
            while ((!exitg1) && (i < 4)) {
              if (!rtIsNaN(delta_v_standing_abs[i - 1])) {
                partialTrueCount = i;
                exitg1 = true;
              } else {
                i++;
              }
            }
          }

          if (partialTrueCount == 0) {
            v_t_yaw_rate = delta_v_standing_abs[0];
            partialTrueCount = 1;
          } else {
            v_t_yaw_rate = delta_v_standing_abs[partialTrueCount - 1];
            b_trueCount = partialTrueCount + 1;
            for (i = b_trueCount; i < 4; i++) {
              vr_ego_unamb_tmp = delta_v_standing_abs[i - 1];
              if (v_t_yaw_rate > vr_ego_unamb_tmp) {
                v_t_yaw_rate = vr_ego_unamb_tmp;
                partialTrueCount = i;
              }
            }
          }

          if (v_t_yaw_rate < 0.6) {
            meas_list->meas[idx1_meas] = 0.0;

            //  ignore stationary locations
            meas_list->alpha[idx1_meas] += 1.5707963267948966 * (static_cast<
              double>(partialTrueCount) - 1.0) + -1.5707963267948966;
          }

          //  if ego-motion compensation not good enough (i.e. moving target), do nothing 
        }
      }

      if (std::abs(meas_list->vr[idx1_meas]) < 0.02) {
        //  probably a ground reflex
        meas_list->meas[idx1_meas] = 0.0;
      }
    }
  }
}

//
// File trailer for preproc_plausi.cpp
//
// [EOF]
//
