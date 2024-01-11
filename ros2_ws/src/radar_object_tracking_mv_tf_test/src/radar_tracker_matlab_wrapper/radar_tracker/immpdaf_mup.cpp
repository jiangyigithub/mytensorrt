//
// File: immpdaf_mup.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "immpdaf_mup.h"
#include "det.h"
#include "ekf_6D_CA_state2meas_obj_offset.h"
#include "ekf_6D_CT_state2meas_obj_offset.h"
#include "mgmt_init_track.h"
#include "mrdivide_helper.h"
#include "radar_tracker.h"
#include "radar_tracker_data.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

//lnl, brake to stop issue
#include <iostream>



// Function Definitions

//
// IMM mixer
//  and Kalman-filter measurement update
//
//  calculate pda weights and perform update for each combination of
//  associated measurement and cell of the local gridmap individually
//
//  y = [dr,alpha,vr]'
//  x = [dx vx ax dy vy ay]'
//
//  Input:
//  IMM:
//    IMM(i).x: state vector of model i
//    IMM(i).P: state covariance of model i
//    IMM(i).C: posterior probability for model i
//
//
//  Output:
//  IMM: Struktur
//    IMM(i).mu: probability of model i
// Arguments    : OBJECT_STRUCT *obj
//                e_struct_T asso_list[20]
//                const double sensor_pos_offset[2]
//                double vx_ego
// Return Type  : void
//
void immpdaf_mup(OBJECT_STRUCT *obj, e_struct_T asso_list[20], const double
                 sensor_pos_offset[2], double vx_ego)
{
  double Ny;
  int i;
  int i1;
  coder::array<double, 3U> asso_likelihood;
  int r1;
  int i2;
  int m;
  int i3;
  int rtemp;
  double a[441];
  int nx;
  double a21;
  int ny;
  int jcol;
  double asso_posterior[8820];
  double expected_meas_CA[3];
  double maxval;
  int r2;
  int r3;
  double angle;
  double delta_vr_delta_psi_tmp;
  double S[9];
  bool x[20];
  double Y[3];
  double state_vector_offset_linear[6];
  double state_vector_offset_turnrate[6];
  double state_vector[6];
  double H[18];
  double current_meas_expected[3];
  signed char S_tmp[3];
  double A[9];
  double K[18];
  double b_H[18];
  signed char b_I[36];
  double c_I[36];
  double d_I[36];




  //  array sizes
  //  number of x-cells in local grid
  Ny = obj->grid.y_len; 
  
  //  number of y-cells in local grid
  //  max number of associated locations
  //  detection probability parameters
  //  calculate update weight
  i = static_cast<int>(obj->grid.x_len);
  i1 = static_cast<int>(obj->grid.y_len);
  //lnl, brake to stop issue
  // std::cout<<"0top_immpdaf:"<<"xlen_i: "<<i<<" ylen_il: "<<i1<<std::endl;
  // std::cout<<"0top_immpdaf:"<<"IMM[0].[0-5]: "<<obj->IMM[0].x[0]<<" "<<obj->IMM[0].x[1]<<" "<<obj->IMM[0].x[2]<<" "<<obj->IMM[0].x[3]<<" "<<obj->IMM[0].x[4]<<" "<<obj->IMM[0].x[5]<<" "<<std::endl;
  // std::cout<<"0top_immpdaf:"<<"IMM[1].[0-5]: "<<obj->IMM[1].x[0]<<" "<<obj->IMM[1].x[1]<<" "<<obj->IMM[1].x[2]<<" "<<obj->IMM[1].x[3]<<" "<<obj->IMM[1].x[4]<<" "<<obj->IMM[1].x[5]<<" "<<std::endl;
  // std::cout<<"0top_immpdaf:"<<" obj.psi: "<<obj->psi<<" pexist: "<<obj->pexist<<" standing: "<<obj->standing<<" RCS_filt: "<<obj->RCS_filt<<" Pgr_reflex: "<<obj->Pgr_reflex<<std::endl;
  // std::cout<<"0top_immpdaf:"<<" asso_list 0: "<<asso_list[0].y[0]<<" "<<asso_list[0].y[1]<<" "<<asso_list[0].y[2]<<" "<<asso_list[0].R[1]<<std::endl;
  // std::cout<<"0top_immpdaf:"<<" asso_list 1: "<<asso_list[1].y[0]<<" "<<asso_list[1].y[1]<<" "<<asso_list[1].y[2]<<" "<<asso_list[1].R[1]<<std::endl;
  // std::cout<<"0top_immpdaf:"<<" asso_list 2: "<<asso_list[2].y[0]<<" "<<asso_list[2].y[1]<<" "<<asso_list[2].y[2]<<" "<<asso_list[2].R[1]<<std::endl;


  asso_likelihood.set_size(i, i1, 20);
  r1 = i * i1 * 20;
  for (i2 = 0; i2 < r1; i2++) {
    asso_likelihood[i2] = 0.0;
  }

  for (m = 0; m < 20; m++) {
    //  bad matlab programming style, but ok for c-code generation
    if (!(asso_list[m].loc_nr == 0.0)) {
      if (0 <= i - 1) {
        i3 = i1;
        if (0 <= i1 - 1) {
          a21 = std::abs(asso_list[m].R[1]);
        }
      }

      for (nx = 0; nx < i; nx++) {
        for (ny = 0; ny < i3; ny++) {
          ekf_6D_CA_state2meas_obj_offset(obj->x, sensor_pos_offset[0],
            sensor_pos_offset[1], vx_ego, obj->grid.x_cells[nx],
            obj->grid.y_cells[ny], obj->psi, expected_meas_CA);

          //lnl, brake to stop issue
          // std::cout<<" 0X_immpdaf_expected_meas 0, 1, 2: "<<expected_meas_CA[0]<<" "<<expected_meas_CA[1]<<" "<<expected_meas_CA[2]<<std::endl;
          // std::cout<<" 0X_immpdaf_asso list 0, 1, 2: "<<"m: "<<m<<" "<<asso_list[m].y[0]<<" "<<asso_list[m].y[1]<<" "<<asso_list[m].y[2]<<std::endl;
          expected_meas_CA[0] = asso_list[m].y[0] - expected_meas_CA[0];
          expected_meas_CA[1] = asso_list[m].y[1] - expected_meas_CA[1];
          expected_meas_CA[2] = asso_list[m].y[2] - expected_meas_CA[2];

          //lnl, brake to stop issue
          // std::cout<<" 0_immpdaf_expected_meas 0, 1, 2: "<<expected_meas_CA[0]<<" "<<expected_meas_CA[1]<<" "<<expected_meas_CA[2]<<std::endl;

          //  substitute for matlab's wrapToPi, which is not supported for code
          //  generation
          for (angle = expected_meas_CA[1]; angle < -3.1415926535897931; angle +=
               6.2831853071795862) {
          }

          while (angle > 3.1415926535897931) {
            angle -= 6.2831853071795862;
          }

          std::memcpy(&S[0], &asso_list[m].R[0], 9U * sizeof(double));
          r1 = 0;
          r2 = 1;
          r3 = 2;
          maxval = std::abs(asso_list[m].R[0]);
          if (a21 > maxval) {
            maxval = a21;
            r1 = 1;
            r2 = 0;
          }

          if (std::abs(asso_list[m].R[2]) > maxval) {
            r1 = 2;
            r2 = 1;
            r3 = 0;
          }

          S[r2] = asso_list[m].R[r2] / asso_list[m].R[r1];
          S[r3] /= S[r1];
          S[r2 + 3] -= S[r2] * S[r1 + 3];
          S[r3 + 3] -= S[r3] * S[r1 + 3];
          S[r2 + 6] -= S[r2] * S[r1 + 6];
          S[r3 + 6] -= S[r3] * S[r1 + 6];
          if (std::abs(S[r3 + 3]) > std::abs(S[r2 + 3])) {
            rtemp = r2;
            r2 = r3;
            r3 = rtemp;
          }

          S[r3 + 3] /= S[r2 + 3];
          S[r3 + 6] -= S[r3 + 3] * S[r2 + 6];
          Y[r1] = expected_meas_CA[0] / S[r1];
          Y[r2] = angle - Y[r1] * S[r1 + 3];
          Y[r3] = expected_meas_CA[2] - Y[r1] * S[r1 + 6];
          Y[r2] /= S[r2 + 3];
          Y[r3] -= Y[r2] * S[r2 + 6];
          Y[r3] /= S[r3 + 6];
          Y[r2] -= Y[r3] * S[r3 + 3];
          Y[r1] -= Y[r3] * S[r3];
          Y[r1] -= Y[r2] * S[r2];
          asso_likelihood[(nx + asso_likelihood.size(0) * ny) +
            asso_likelihood.size(0) * asso_likelihood.size(1) * m] = std::exp
            ((-0.5 * Y[0] * expected_meas_CA[0] + -0.5 * Y[1] * angle) + -0.5 *
             Y[2] * expected_meas_CA[2]);
        }
      }
    }
  }

  //  gridmap is association prior, to be combined with above likelihood
  //  convert odds to prob
  for (i1 = 0; i1 < 441; i1++) {
    a[i1] = obj->grid.LR[i1] / (obj->grid.LR[i1] + 1.0);
  }

  for (rtemp = 0; rtemp < 20; rtemp++) {
    r1 = rtemp * 441 - 1;
    for (jcol = 0; jcol < 21; jcol++) {
      r2 = jcol * 21;
      r3 = r1 + jcol * 21;
      std::memcpy(&asso_posterior[r3 + 1], &a[r2], 21U * sizeof(double));
    }
  }

  for (i1 = 0; i1 < 8820; i1++) {
    asso_posterior[i1] *= asso_likelihood[i1];
  }

  //  normalize
  maxval = asso_posterior[0];
  for (r1 = 0; r1 < 8819; r1++) {
    maxval += asso_posterior[r1 + 1];
  }

  if (maxval == 0.0) {
    //  association is really bad.
    obj->meas = false;
    obj->meas_angles = 0.0;
    for (i = 0; i < 20; i++) {
      asso_list[i] = r;
    }

    //  an empty asso-list element
  } else {
    int b_i;

    //  set small probabilities to zero and renorm
    for (b_i = 0; b_i < 8820; b_i++) {
      delta_vr_delta_psi_tmp = asso_posterior[b_i] / maxval;
      asso_posterior[b_i] = delta_vr_delta_psi_tmp;
      if (delta_vr_delta_psi_tmp < 0.0005) {
        asso_posterior[b_i] = 0.0;
      }
    }

    maxval = asso_posterior[0];
    for (r1 = 0; r1 < 8819; r1++) {
      maxval += asso_posterior[r1 + 1];
    }

    for (i1 = 0; i1 < 8820; i1++) {
      asso_posterior[i1] /= maxval;
    }

    //  update each combination of grid cell and associaited tearget individually. 
    //  model probability update -> see Blackman/Popoli pp 226ff
    expected_meas_CA[0] = 0.00079999999999999982;
    expected_meas_CA[1] = 0.00079999999999999982;
    expected_meas_CA[2] = 0.00079999999999999982;
    for (nx = 0; nx < i; nx++) {
      i1 = static_cast<int>(Ny);
      for (ny = 0; ny < i1; ny++) {
        bool y;
        bool exitg1;

        //  quick reject low proababilities
        for (i2 = 0; i2 < 20; i2++) {
          x[i2] = (asso_posterior[(nx + 21 * ny) + 441 * i2] < 0.0005);
        }

        y = true;
        r1 = 0;
        exitg1 = false;
        while ((!exitg1) && (r1 < 20)) {
          if (!x[r1]) {
            y = false;
            exitg1 = true;
          } else {
            r1++;
          }
        }

        if (!y) {
          double x_offset_obj;
          double y_offset_obj;
          x_offset_obj = obj->grid.x_cells[nx];
          y_offset_obj = obj->grid.y_cells[ny];

          //  represent local grid map element as state vector
          a21 = std::sin(-obj->psi);
          maxval = std::cos(-obj->psi);

          //  state_vector_offset = + grid_offset - sensor_position_offset
          angle = (maxval * obj->grid.x_cells[nx] + a21 * obj->grid.y_cells[ny])
            - sensor_pos_offset[0];
          state_vector_offset_linear[0] = angle;
          state_vector_offset_linear[1] = 0.0;
          state_vector_offset_linear[2] = 0.0;
          a21 = (-a21 * obj->grid.x_cells[nx] + maxval * obj->grid.y_cells[ny])
            - sensor_pos_offset[1];
          state_vector_offset_linear[3] = a21;
          state_vector_offset_linear[4] = 0.0;
          state_vector_offset_linear[5] = 0.0;

          //  for IMM(2) and IMM(3)
          state_vector_offset_turnrate[0] = angle;
          state_vector_offset_turnrate[1] = a21;
          state_vector_offset_turnrate[2] = 0.0;
          state_vector_offset_turnrate[3] = 0.0;
          state_vector_offset_turnrate[4] = 0.0;
          state_vector_offset_turnrate[5] = 0.0;

          //  for IMM(1)
          for (m = 0; m < 20; m++) {
            if (!(asso_list[m].loc_nr == 0.0)) {
              //  quick reject low proababilities
              i2 = (nx + 21 * ny) + 441 * m;
              if (!(asso_posterior[i2] < 0.0005)) {
                for (b_i = 0; b_i < 3; b_i++) {
                  //  calculate state Jacobian H and measurement residual e
                  if (b_i + 1 == 1) {
                    //  Jacobi-Matrix
                    for (i3 = 0; i3 < 6; i3++) {
                      state_vector[i3] = obj->IMM[0].x[i3] +
                        state_vector_offset_turnrate[i3];
                    }

                    double range;
                    double H_tmp;
                    double H_tmp_tmp;

                    // EKF_6D_CA_GETJACOBIAN get Jacobian for EKF update
                    //  derivative of measurement for state vector (delta y)/(delta x) 
                    //  state vector = [px, py, heading, turn_rate, v_abs, 0].'
                    //  measurement = [range, alpha, v_radial].'
                    //  r = sqrt(px^2 + py^2)
                    //  alpha = atan2( py/px )
                    //  vr = (px*vx + py*vy) / r
                    //  vx = v_abs * cos(heading)
                    //  vy = v_abs * sin(heading)
                    a21 = state_vector[0] * state_vector[0];
                    maxval = state_vector[1] * state_vector[1];
                    range = std::sqrt(a21 + maxval);

                    //  delta_r_delta_px = px / r
                    //  delta_r_delta_py = py / r
                    //  delta_alpha_delta_px = -py/r^2
                    //  delta_alpha_delta_py = px/r^2
                    //  delta_vr_delta_px = vs*cos(psi)/r - vs*cos(psi)*px^2/r^3 
                    angle = std::cos(state_vector[2]);

                    //  delta_vr_delta_py = vs*cos(psi)/r - vs*cos(psi)*py^2/r^3 
                    //  delta_vr_delta_psi = (-sin(psi)*vs*px + cos(psi)*vs*py) / r   /lnl ????
                    delta_vr_delta_psi_tmp = std::sin(state_vector[2]);

                    //  delta_vr_delta_vs = (px*cos(psi + py*sin(psi)) / r
                    H[0] = state_vector[0] / range;
                    H[3] = state_vector[1] / range;
                    H[6] = 0.0;
                    H[9] = 0.0;
                    H[12] = 0.0;
                    H[15] = 0.0;
                    H_tmp = range * range;
                    H[1] = -state_vector[1] / H_tmp;
                    H[4] = state_vector[0] / H_tmp;
                    H[7] = 0.0;
                    H[10] = 0.0;
                    H[13] = 0.0;
                    H[16] = 0.0;
                    H_tmp_tmp = state_vector[4] * angle;
                    H_tmp = rt_powd_snf(range, 3.0);
                    H[2] = H_tmp_tmp / range - H_tmp_tmp * a21 / H_tmp;
                    H[5] = state_vector[4] * std::cos(state_vector[2]) / range -
                      H_tmp_tmp * maxval / H_tmp;
                    H[8] = state_vector[4] * (-delta_vr_delta_psi_tmp *
                      state_vector[0] + angle * state_vector[1]) / range;
                    H[11] = 0.0;
                    H[14] = (state_vector[0] * angle + state_vector[1] *
                             delta_vr_delta_psi_tmp) / range;
                    H[17] = 0.0;
                    ekf_6D_CT_state2meas_obj_offset(obj->IMM[0].x,
                      sensor_pos_offset[0], sensor_pos_offset[1], vx_ego,
                      x_offset_obj, y_offset_obj, obj->psi,
                      current_meas_expected);
                    //lnl, brake to stop 
                    // std::cout<<" 0AA_immpdaf_expected_meas 0, 1, 2: "<<current_meas_expected[0]<<" "<<current_meas_expected[1]<<" "<<current_meas_expected[2]<<std::endl;
                    // std::cout<<" 0AA_immpdaf_asso list 0, 1, 2: "<<"m: "<<m<<" "<<asso_list[m].y[0]<<" "<<asso_list[m].y[1]<<" "<<asso_list[m].y[2]<<std::endl;

                    //  measurement residual vector
                    current_meas_expected[0] = asso_list[m].y[0] -
                      current_meas_expected[0];
                    current_meas_expected[1] = asso_list[m].y[1] -
                      current_meas_expected[1];
                    current_meas_expected[2] = asso_list[m].y[2] -
                      current_meas_expected[2];
                  } else {
                    for (i3 = 0; i3 < 6; i3++) {
                      state_vector[i3] = obj->IMM[b_i].x[i3] +
                        state_vector_offset_linear[i3];
                    }

                    double range;
                    double H_tmp;

                    // EKF_6D_CA_GETJACOBIAN get Jacobian for EKF update
                    //  derivative of measurement for state vector (delta y)/(delta x) 
                    //  !!! (px, py) must be already adjusted for sensor position and grid offset 
                    //
                    //  state vector = [px, vx, ax, py, vy, ay].'
                    //  measurement = [range, alpha, v_radial].'
                    //  r = sqrt(px^2 + py^2)
                    //  alpha = atan2( py/px )
                    //  vr = (px*vx + py*vy) / r
                    range = std::sqrt(state_vector[0] * state_vector[0] +
                                      state_vector[3] * state_vector[3]);

                    //  delta_r_delta_px = px / r
                    //  delta_r_delta_py = py / r
                    //  delta_alpha_delta_px = -py/r^2
                    //  delta_alpha_delta_py = px/r^2
                    //  delta_vr_delta_px = vx/r - px*(px*vx + py*vy)/r^3
                    //  delta_vr_delta_py = vy/r - py*(px*vx + py*vy)/r^3
                    //  delta_vr_delta_vx = px/r
                    //  delta_vr_delta_vy = py/r
                    H_tmp = state_vector[0] / range;
                    H[0] = H_tmp;
                    H[3] = 0.0;
                    H[6] = 0.0;
                    a21 = state_vector[3] / range;
                    H[9] = a21;
                    H[12] = 0.0;
                    H[15] = 0.0;
                    maxval = range * range;
                    H[1] = -state_vector[3] / maxval;
                    H[4] = 0.0;
                    H[7] = 0.0;
                    H[10] = state_vector[0] / maxval;
                    H[13] = 0.0;
                    H[16] = 0.0;
                    maxval = state_vector[0] * state_vector[1] + state_vector[3]
                      * state_vector[4];
                    angle = rt_powd_snf(range, 3.0);
                    H[2] = state_vector[1] / range - state_vector[0] * maxval /
                      angle;
                    H[5] = H_tmp;
                    H[8] = 0.0;
                    H[11] = state_vector[4] / range - state_vector[3] * maxval /
                      angle;
                    H[14] = a21;
                    H[17] = 0.0;
                    ekf_6D_CA_state2meas_obj_offset(obj->IMM[b_i].x,
                      sensor_pos_offset[0], sensor_pos_offset[1], vx_ego,
                      x_offset_obj, y_offset_obj, obj->psi,
                      current_meas_expected);
                    //lnl, brake to stop 
                    // std::cout<<" 0BB_immpdaf_expected_meas 0, 1, 2: "<<current_meas_expected[0]<<" "<<current_meas_expected[1]<<" "<<current_meas_expected[2]<<std::endl;
                    // std::cout<<" 0BB_immpdaf_asso list 0, 1, 2: "<<"m: "<<m<<" "<<asso_list[m].y[0]<<" "<<asso_list[m].y[1]<<" "<<asso_list[m].y[2]<<std::endl;
                    current_meas_expected[0] = asso_list[m].y[0] -
                      current_meas_expected[0];
                    current_meas_expected[1] = asso_list[m].y[1] -
                      current_meas_expected[1];   //lnl, alpha should have minutes
                    current_meas_expected[2] = asso_list[m].y[2] -
                      current_meas_expected[2];
                  }

                  //  substitute for matlab's wrapToPi, which is not supported for code 
                  //  generation   //lnl, need to check !!!
                  for (angle = current_meas_expected[1]; angle <
                       -3.1415926535897931; angle += 6.2831853071795862) {
                  }

                  while (angle > 3.1415926535897931) {
                    angle -= 6.2831853071795862;
                  }

                  //  calculate statistical distance
                  for (i3 = 0; i3 < 3; i3++) {
                    S_tmp[i3] = static_cast<signed char>(i3 + 1);
                    for (r2 = 0; r2 < 6; r2++) {
                      r1 = i3 + 3 * r2;
                      K[r2 + 6 * i3] = H[r1];
                      delta_vr_delta_psi_tmp = 0.0;
                      for (jcol = 0; jcol < 6; jcol++) {
                        delta_vr_delta_psi_tmp += H[i3 + 3 * jcol] * obj->
                          IMM[b_i].P[jcol + 6 * r2];
                      }

                      b_H[r1] = delta_vr_delta_psi_tmp;
                    }
                  }

                  for (i3 = 0; i3 < 3; i3++) {
                    for (r2 = 0; r2 < 3; r2++) {
                      delta_vr_delta_psi_tmp = 0.0;
                      for (jcol = 0; jcol < 6; jcol++) {
                        delta_vr_delta_psi_tmp += b_H[i3 + 3 * jcol] * K[jcol +
                          6 * r2];
                      }

                      S[i3 + 3 * r2] = delta_vr_delta_psi_tmp + asso_list[m].R
                        [(S_tmp[i3] + 3 * (S_tmp[r2] - 1)) - 1];
                    }
                  }

                  //  measurement residual covariance matrix
                  std::memcpy(&A[0], &S[0], 9U * sizeof(double));
                  r1 = 0;
                  r2 = 1;
                  r3 = 2;
                  maxval = std::abs(S[0]);
                  a21 = std::abs(S[1]);
                  if (a21 > maxval) {
                    maxval = a21;
                    r1 = 1;
                    r2 = 0;
                  }

                  if (std::abs(S[2]) > maxval) {
                    r1 = 2;
                    r2 = 1;
                    r3 = 0;
                  }

                  A[r2] = S[r2] / S[r1];
                  A[r3] /= A[r1];
                  A[r2 + 3] -= A[r2] * A[r1 + 3];
                  A[r3 + 3] -= A[r3] * A[r1 + 3];
                  A[r2 + 6] -= A[r2] * A[r1 + 6];
                  A[r3 + 6] -= A[r3] * A[r1 + 6];
                  if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
                    rtemp = r2;
                    r2 = r3;
                    r3 = rtemp;
                  }

                  A[r3 + 3] /= A[r2 + 3];
                  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
                  Y[r1] = current_meas_expected[0] / A[r1];
                  Y[r2] = angle - Y[r1] * A[r1 + 3];
                  Y[r3] = current_meas_expected[2] - Y[r1] * A[r1 + 6];
                  Y[r2] /= A[r2 + 3];
                  Y[r3] -= Y[r2] * A[r2 + 6];
                  Y[r3] /= A[r3 + 6];
                  Y[r2] -= Y[r3] * A[r3 + 3];
                  Y[r1] -= Y[r3] * A[r3];
                  Y[r1] -= Y[r2] * A[r2];
                  maxval = (Y[0] * current_meas_expected[0] + Y[1] * angle) + Y
                    [2] * current_meas_expected[2];

                  //  Mahalanobis distance
                  a21 = 0.063493635934240969 * rt_powd_snf(det(S), -0.5) * std::
                    exp(-maxval / 2.0);

                  //lnl, brake to stop issue
                  // std::cout<<" 1_immpdaf_a21: "<<a21<<std::endl;

                  //  sanity check. Sometimes d2 is too large
                  if ((!(a21 == rtInf)) && ((!rtIsInf(a21)) && (!rtIsNaN(a21))))
                  {
                    //  PDA-formula for likelihood
                    expected_meas_CA[b_i] += 0.8 * a21;

                    //lnl, brake to stop issue
                    // std::cout<<" 2_immpdaf_"<<"expected_meas_CA["<<b_i<<"]: "<<expected_meas_CA[b_i]<<std::endl;

                    //  will be normalized later
                    //  perform Kalman update
                    for (i3 = 0; i3 < 6; i3++) {
                      for (r2 = 0; r2 < 3; r2++) {
                        delta_vr_delta_psi_tmp = 0.0;
                        for (jcol = 0; jcol < 6; jcol++) {
                          delta_vr_delta_psi_tmp += obj->IMM[b_i].P[i3 + 6 *
                            jcol] * K[jcol + 6 * r2];
                        }

                        b_H[i3 + 6 * r2] = delta_vr_delta_psi_tmp;
                      }
                    }

                    b_mrdiv(b_H, S, K);
                    for (i3 = 0; i3 < 18; i3++) {
                      K[i3] *= asso_posterior[i2];
                    }

                    //  weight by association probability 
                    // lnl, need check
                    if (b_i ==0 )
                    {
                      if (std::abs(obj->IMM[0].x[3]>0.1))
                        std::cout<<"0A_immpdaf-"<<"current_meas_expected[0]: "<<current_meas_expected[0]<<" current_meas_expected[1]: " 
                        <<current_meas_expected[1]<<" current_meas_expected[2]: "<<current_meas_expected[2]<<std::endl;
                    }
                    
                    for (i3 = 0; i3 < 6; i3++) {
                      
                      if (b_i==0&&i3==3)
                      {
                        if (std::abs(obj->IMM[b_i].x[i3]>0.1))
                          std::cout<<"0B_immpdaf-"<<" before obj->IMM["<<b_i<<"].x["<<i3<<"]: "<<obj->IMM[b_i].x[i3]<<std::endl;
                      }

                      obj->IMM[b_i].x[i3] += (K[i3] * current_meas_expected[0] +
                        K[i3 + 6] * angle) + K[i3 + 12] * current_meas_expected
                        [2];
                      
                      // if (b_i==0&&i3<5)
                      // {
                      //   std::cout<<"0D_immpdaf-"<<" after obj->IMM["<<b_i<<"].x["<<i3<<"]: "<<obj->IMM[b_i].x[i3]<<std::endl;
                      // }

                      if (b_i==0&&i3==3)
                      {
                        
                        if (std::abs(obj->IMM[b_i].x[i3])>0.01)
                        {
                          auto test = (K[i3] * current_meas_expected[0] +
                        K[i3 + 6] * angle) + K[i3 + 12] * current_meas_expected
                        [2];
                          std::cout<<"1C_immpdaf-0"<<"current_meas_expected[0]: "<<current_meas_expected[0]<<" current_meas_expected[1]: " 
                          <<current_meas_expected[1]<<" current_meas_expected[2]: "<<current_meas_expected[2]<<
                          " Factors: "<<"K[i3]: "<<K[i3]<<" K[i3+6] key: "<<K[i3+6]<<" K[i3+12]: "<<K[i3+12] 
                          <<" angle: "<<angle<<" test: "<<test<<" after obj->IMM["<<b_i<<"].x["<<i3<<"]: "<<obj->IMM[b_i].x[i3]<<std::endl;

                        }
                      }
                      
                    }

                    for (i3 = 0; i3 < 36; i3++) {
                      b_I[i3] = 0;
                    }

                    for (r1 = 0; r1 < 6; r1++) {
                      b_I[r1 + 6 * r1] = 1;
                    }

                    for (i3 = 0; i3 < 6; i3++) {
                      delta_vr_delta_psi_tmp = K[i3 + 6];
                      a21 = K[i3 + 12];
                      for (r2 = 0; r2 < 6; r2++) {
                        r1 = i3 + 6 * r2;
                        d_I[r1] = static_cast<double>(b_I[r1]) - ((K[i3] * H[3 *
                          r2] + delta_vr_delta_psi_tmp * H[3 * r2 + 1]) + a21 *
                          H[3 * r2 + 2]);
                      }

                      for (r2 = 0; r2 < 6; r2++) {
                        delta_vr_delta_psi_tmp = 0.0;
                        for (jcol = 0; jcol < 6; jcol++) {
                          delta_vr_delta_psi_tmp += d_I[i3 + 6 * jcol] *
                            obj->IMM[b_i].P[jcol + 6 * r2];
                        }

                        c_I[i3 + 6 * r2] = delta_vr_delta_psi_tmp;
                      }
                    }

                    std::memcpy(&obj->IMM[b_i].P[0], &c_I[0], 36U * sizeof
                                (double));
                    a21 = asso_posterior[i2];
                    for (i3 = 0; i3 < 3; i3++) {
                      r2 = 3 * i3 + 9 * b_i;
                      delta_vr_delta_psi_tmp = asso_list[m].S[r2] + S[3 * i3] *
                        a21;
                      S[3 * i3] = delta_vr_delta_psi_tmp;
                      asso_list[m].S[r2] = delta_vr_delta_psi_tmp;
                      jcol = 3 * i3 + 1;
                      r1 = r2 + 1;
                      delta_vr_delta_psi_tmp = asso_list[m].S[r1] + S[jcol] *
                        a21;
                      S[jcol] = delta_vr_delta_psi_tmp;
                      asso_list[m].S[r1] = delta_vr_delta_psi_tmp;
                      jcol = 3 * i3 + 2;
                      r2 += 2;
                      delta_vr_delta_psi_tmp = asso_list[m].S[r2] + S[jcol] *
                        a21;
                      S[jcol] = delta_vr_delta_psi_tmp;
                      asso_list[m].S[r2] = delta_vr_delta_psi_tmp;
                    }

                    asso_list[m].d2[b_i] += maxval * asso_posterior[i2];
                  }
                }
              }
            } else {
              //  association list entry is empty
            }
          }
        }
      }
    }

    while (obj->IMM[0].x[2] < -3.1415926535897931) {
      obj->IMM[0].x[2] += 6.2831853071795862;
    }

    while (obj->IMM[0].x[2] > 3.1415926535897931) {
      obj->IMM[0].x[2] -= 6.2831853071795862;
    }

    //  update model probabilies
    //  norming factor -> see Blackman/Popoli p 227

    maxval = expected_meas_CA[0] * obj->IMM[0].mu;

    //lnl, brake to stop issue. update 2022. to improve turning case
    if(obj->standing > 0.5 || ((std::abs(obj->IMM[0].x[3])<0.05)&&(std::abs(obj->IMM[0].x[3])>0)&&(std::abs(obj->IMM[0].x[4]< 3))))
    {
      // maxval = std::pow((0.1-std::abs(obj->IMM[0].x[3])), 16)*maxval;
      maxval = std::pow((std::abs(obj->IMM[0].x[3])/0.05),3)*maxval;
    }   
    else if((std::abs(obj->IMM[0].x[3])>=0.1)&&(std::abs(obj->IMM[0].x[4]> 6)&&(std::abs(obj->IMM[0].x[4] < 12))))
    {
      maxval = std::pow(1.5,((std::abs(obj->IMM[0].x[3])-0.1)/0.1))*maxval;
    }
    else if((std::abs(obj->IMM[0].x[3])>0.05)&&(std::abs(obj->IMM[0].x[3])<0.15)&&(std::abs(obj->IMM[0].x[4] >= 12)))
    {
      maxval = std::pow(((std::abs(obj->IMM[0].x[3])-0.05))/(0.15-0.05),3)*maxval;
    }
    // maxval = std::pow(1.3,((std::abs(obj->IMM[0].x[3]))/0.1))*maxval;

    a21 = (maxval + expected_meas_CA[1] * obj->IMM[1].mu) + expected_meas_CA[2] *
      obj->IMM[2].mu;

    // std::cout<<" 3_immpdaf_a21: "<<a21<<std::endl;
    //  calculate model-probability
    obj->IMM[0].mu = maxval / a21;
    obj->IMM[1].mu = expected_meas_CA[1] * obj->IMM[1].mu / a21;
    obj->IMM[2].mu = expected_meas_CA[2] * obj->IMM[2].mu / a21;



    //lnl, brake to stop issue
    std::cout<<" 3_immpdaf_IMM[0, 1, 2].mu: "<< obj->IMM[0].mu<<" "<<obj->IMM[1].mu<<" "<<obj->IMM[2].mu<<std::endl;
    // std::cout<<" 3_immpdaf_IMM[0].x[1]: "<< obj->IMM[0].x[1]<<std::endl;
    if (std::abs(obj->IMM[0].x[3])>0.001)
      std::cout<<"0top_immpdaf:"<<"mu: "<< obj->IMM[0].mu<<" "<<obj->IMM[1].mu<<" "<<obj->IMM[2].mu<<" IMM[0].[0-5]: "<<obj->IMM[0].x[0]<<" "<<obj->IMM[0].x[1]<<" "<<obj->IMM[0].x[2]<<" turning_rate: "<<obj->IMM[0].x[3]<<" "<<obj->IMM[0].x[4]<<" "<<obj->IMM[0].x[5]<<" "<<std::endl;
 
 

  }
}

//
// File trailer for immpdaf_mup.cpp
//
// [EOF]
//
