//
// File: imm_predict.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "imm_predict.h"
#include "mgmt_init_track.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Modell 1:
//  x = [dx,dy,h,omega,s, 0]'
//  Modell 2:
//  6D Kalman predction (linear motion)
//  x = [dx vx ax dy vy ay]'
// Arguments    : OBJECT_STRUCT *obj
//                double dt
// Return Type  : void
//
void imm_predict(OBJECT_STRUCT *obj, double dt)
{
  double A[36];
  double b_A[36];
  double BW;
  double Q[36];
  static const signed char iv[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv1[6] = { 0, 0, 0, 0, 0, 1 };

  static const signed char iv2[6] = { 0, 0, 0, 1, 0, 0 };

  static const signed char iv3[6] = { 0, 1, 0, 0, 0, 0 };

  static const signed char iv4[6] = { 0, 0, 0, 0, 1, 0 };

  double c_A[6];
  double d_A[36];
  if (std::abs(dt) > 0.0) {
    for (int imm_idx = 0; imm_idx < 3; imm_idx++) {
      if (imm_idx + 1 == 1) {
        double A_tmp;
        double vx;
        double vy_tmp;
        double vy;
        double SW;
        double CW;
        double var_omega;
        double AW;

        //  Constant turn rate model
        std::memcpy(&A[0], &obj->IMM[imm_idx].P[0], 36U * sizeof(double));

        //  x = [dx,dy,h,omega,s, 0]'
        A_tmp = std::cos(obj->IMM[imm_idx].x[2]);
        vx = obj->IMM[imm_idx].x[4] * A_tmp;
        vy_tmp = std::sin(obj->IMM[imm_idx].x[2]);
        vy = obj->IMM[imm_idx].x[4] * vy_tmp;
        if (std::abs(obj->IMM[imm_idx].x[3]) > 0.0) {
          BW = obj->IMM[imm_idx].x[3] * dt;
          var_omega = std::sin(BW);
          SW = var_omega / BW;
          AW = std::cos(BW);
          CW = (1.0 - AW) / BW;
          AW = 1.0 / obj->IMM[imm_idx].x[3] * (AW - SW);
          BW = 1.0 / obj->IMM[imm_idx].x[3] * (var_omega - CW);
        } else {
          SW = 1.0;
          CW = 0.0;
          AW = 0.0;
          BW = 0.5 * dt;
        }

        //  state-prediction
        if (obj->standing > 0.5) {
          obj->IMM[imm_idx].x[0] += vx * dt;
          obj->IMM[imm_idx].x[1] += vy * dt;
          obj->IMM[imm_idx].x[3] = 0.0;
          obj->IMM[imm_idx].x[5] = 0.0;
        } else {
          obj->IMM[imm_idx].x[0] += dt * (SW * vx - CW * vy);
          obj->IMM[imm_idx].x[1] += dt * (CW * vx + SW * vy);
          obj->IMM[imm_idx].x[2] += obj->IMM[imm_idx].x[3] * dt;
          obj->IMM[imm_idx].x[5] = 0.0;
        }

        //  cov-prediction
        if (dt > 0.0) {
          double var_s;
          int i;
          if (obj->standing > 0.5) {
            var_omega = 0.010000000000000002;
            var_s = 0.25;
          } else {
            var_omega = 0.25;
            var_s = 1.0;
          }

          b_A[0] = 1.0;
          b_A[6] = 0.0;
          b_A[12] = -dt * (SW * vy + CW * vx);
          b_A[18] = dt * (AW * vx - BW * vy);
          b_A[24] = dt * (SW * A_tmp - CW * vy_tmp);
          b_A[30] = 0.0;
          b_A[1] = 0.0;
          b_A[7] = 1.0;
          b_A[13] = dt * (SW * vx - CW * vy);
          b_A[19] = dt * (AW * vy + BW * vx);
          b_A[25] = dt * (SW * vy_tmp + CW * A_tmp);
          b_A[31] = 0.0;
          b_A[2] = 0.0;
          b_A[8] = 0.0;
          b_A[14] = 1.0;
          b_A[20] = dt;
          b_A[26] = 0.0;
          b_A[32] = 0.0;
          for (i = 0; i < 6; i++) {
            b_A[6 * i + 3] = iv2[i];
            b_A[6 * i + 4] = iv4[i];
            b_A[6 * i + 5] = 0.0;
          }

          //  1/3*dt^3*var_xy is for consistency with CV/CA
          for (i = 0; i < 6; i++) {
            int i1;
            int c_A_tmp;
            for (i1 = 0; i1 < 6; i1++) {
              BW = 0.0;
              for (c_A_tmp = 0; c_A_tmp < 6; c_A_tmp++) {
                BW += b_A[i + 6 * c_A_tmp] * obj->IMM[imm_idx].P[c_A_tmp + 6 *
                  i1];
              }

              d_A[i + 6 * i1] = BW;
            }

            for (i1 = 0; i1 < 6; i1++) {
              BW = 0.0;
              for (c_A_tmp = 0; c_A_tmp < 6; c_A_tmp++) {
                BW += d_A[i + 6 * c_A_tmp] * b_A[i1 + 6 * c_A_tmp];
              }

              A[i + 6 * i1] = BW;
            }
          }

          A_tmp = 0.33333333333333331 * rt_powd_snf(dt, 3.0) * 0.0625;
          b_A[0] = A_tmp;
          b_A[6] = 0.0;
          b_A[12] = 0.0;
          b_A[18] = 0.0;
          b_A[24] = 0.0;
          b_A[30] = 0.0;
          b_A[1] = 0.0;
          b_A[7] = A_tmp;
          b_A[13] = 0.0;
          b_A[19] = 0.0;
          b_A[25] = 0.0;
          b_A[31] = 0.0;
          b_A[2] = 0.0;
          b_A[8] = 0.0;
          b_A[14] = rt_powd_snf(dt, 3.0) / 3.0 * var_omega + dt *
            0.010000000000000002;
          A_tmp = dt * dt / 2.0 * var_omega;
          b_A[20] = A_tmp;
          b_A[26] = 0.0;
          b_A[32] = 0.0;
          b_A[3] = 0.0;
          b_A[9] = 0.0;
          b_A[15] = A_tmp;
          b_A[21] = dt * var_omega;
          b_A[27] = 0.0;
          b_A[33] = 0.0;
          b_A[4] = 0.0;
          b_A[10] = 0.0;
          b_A[16] = 0.0;
          b_A[22] = 0.0;
          b_A[28] = dt * var_s;
          b_A[34] = 0.0;
          for (i = 0; i < 6; i++) {
            b_A[6 * i + 5] = 0.0;
          }

          for (i = 0; i < 36; i++) {
            A[i] += b_A[i];
          }
        }

        std::memcpy(&obj->IMM[imm_idx].P[0], &A[0], 36U * sizeof(double));
      } else if (imm_idx + 1 == 2) {
        int i;
        int b_A_tmp;
        int i1;
        int c_A_tmp;

        //  Cartesian model high-dyn
        std::memcpy(&A[0], &obj->IMM[imm_idx].P[0], 36U * sizeof(double));

        //  x = [dx,vx,ax,dy,vy,ay]'
        if (obj->standing > 0.5) {
          double AW;

          //  system matrix
          b_A[0] = 1.0;
          b_A[6] = dt;
          b_A[12] = 0.0;
          b_A[18] = 0.0;
          b_A[24] = 0.0;
          b_A[30] = 0.0;
          b_A[3] = 0.0;
          b_A[9] = 0.0;
          b_A[15] = 0.0;
          b_A[21] = 1.0;
          b_A[27] = dt;
          b_A[33] = 0.0;

          //  system noise cov
          BW = 0.25 * (0.33333333333333331 * rt_powd_snf(dt, 3.0));
          Q[0] = BW;
          AW = 0.25 * (0.5 * (dt * dt));
          Q[6] = AW;
          Q[12] = 0.0;
          Q[18] = 0.0;
          Q[24] = 0.0;
          Q[30] = 0.0;
          Q[1] = AW;
          Q[7] = 0.25 * dt;
          Q[13] = 0.0;
          Q[19] = 0.0;
          Q[25] = 0.0;
          Q[31] = 0.0;
          Q[3] = 0.0;
          Q[9] = 0.0;
          Q[15] = 0.0;
          Q[21] = BW;
          Q[27] = AW;
          Q[33] = 0.0;
          Q[4] = 0.0;
          Q[10] = 0.0;
          Q[16] = 0.0;
          Q[22] = AW;
          Q[28] = 0.25 * dt;
          Q[34] = 0.0;
          for (i = 0; i < 6; i++) {
            b_A[6 * i + 1] = iv3[i];
            b_A_tmp = 6 * i + 2;
            b_A[b_A_tmp] = 0.0;
            b_A[6 * i + 4] = iv4[i];
            c_A_tmp = 6 * i + 5;
            b_A[c_A_tmp] = 0.0;
            Q[b_A_tmp] = 0.0;
            Q[c_A_tmp] = 0.0;
          }
        } else {
          double A_tmp;

          //  system matrix
          b_A[0] = 1.0;
          b_A[6] = dt;
          A_tmp = 0.5 * (dt * dt);
          b_A[12] = A_tmp;
          b_A[18] = 0.0;
          b_A[24] = 0.0;
          b_A[30] = 0.0;
          b_A[1] = 0.0;
          b_A[7] = 1.0;
          b_A[13] = dt;
          b_A[19] = 0.0;
          b_A[25] = 0.0;
          b_A[31] = 0.0;
          b_A[3] = 0.0;
          b_A[9] = 0.0;
          b_A[15] = 0.0;
          b_A[21] = 1.0;
          b_A[27] = dt;
          b_A[33] = A_tmp;
          b_A[4] = 0.0;
          b_A[10] = 0.0;
          b_A[16] = 0.0;
          b_A[22] = 0.0;
          b_A[28] = 1.0;
          b_A[34] = dt;
          for (i = 0; i < 6; i++) {
            b_A[6 * i + 2] = iv[i];
            b_A[6 * i + 5] = iv1[i];
          }

          double CW;
          double var_omega;
          double AW;

          //  system noise cov
          BW = 0.05 * rt_powd_snf(dt, 5.0);
          Q[0] = BW;
          AW = 0.125 * rt_powd_snf(dt, 4.0);
          Q[6] = AW;
          var_omega = 0.16666666666666666 * rt_powd_snf(dt, 3.0);
          Q[12] = var_omega;
          Q[18] = 0.0;
          Q[24] = 0.0;
          Q[30] = 0.0;
          Q[1] = AW;
          CW = 0.33333333333333331 * rt_powd_snf(dt, 3.0);
          Q[7] = CW;
          Q[13] = A_tmp;
          Q[19] = 0.0;
          Q[25] = 0.0;
          Q[31] = 0.0;
          Q[2] = var_omega;
          Q[8] = A_tmp;
          Q[14] = dt;
          Q[20] = 0.0;
          Q[26] = 0.0;
          Q[32] = 0.0;
          Q[3] = 0.0;
          Q[9] = 0.0;
          Q[15] = 0.0;
          Q[21] = BW;
          Q[27] = AW;
          Q[33] = var_omega;
          Q[4] = 0.0;
          Q[10] = 0.0;
          Q[16] = 0.0;
          Q[22] = AW;
          Q[28] = CW;
          Q[34] = A_tmp;
          Q[5] = 0.0;
          Q[11] = 0.0;
          Q[17] = 0.0;
          Q[23] = var_omega;
          Q[29] = A_tmp;
          Q[35] = dt;
        }

        //  state prediction
        for (i = 0; i < 6; i++) {
          BW = 0.0;
          for (i1 = 0; i1 < 6; i1++) {
            BW += b_A[i + 6 * i1] * obj->IMM[imm_idx].x[i1];
          }

          c_A[i] = BW;
        }

        for (i = 0; i < 6; i++) {
          obj->IMM[imm_idx].x[i] = c_A[i];
        }

        //  cov prediction
        if (dt > 0.0) {
          //  Kein Cov-Prediktion bei Retrodiktion
          for (i = 0; i < 6; i++) {
            for (i1 = 0; i1 < 6; i1++) {
              BW = 0.0;
              for (c_A_tmp = 0; c_A_tmp < 6; c_A_tmp++) {
                BW += b_A[i + 6 * c_A_tmp] * obj->IMM[imm_idx].P[c_A_tmp + 6 *
                  i1];
              }

              d_A[i + 6 * i1] = BW;
            }

            for (i1 = 0; i1 < 6; i1++) {
              BW = 0.0;
              for (c_A_tmp = 0; c_A_tmp < 6; c_A_tmp++) {
                BW += d_A[i + 6 * c_A_tmp] * b_A[i1 + 6 * c_A_tmp];
              }

              b_A_tmp = i + 6 * i1;
              A[b_A_tmp] = BW + Q[b_A_tmp];
            }
          }
        }

        std::memcpy(&obj->IMM[imm_idx].P[0], &A[0], 36U * sizeof(double));
      } else {
        int i;
        int b_A_tmp;
        int i1;
        int c_A_tmp;

        //  Cartesian model low-dyn
        std::memcpy(&A[0], &obj->IMM[imm_idx].P[0], 36U * sizeof(double));

        //  x = [dx,vx,ax,dy,vy,ay]'
        if (obj->standing > 0.5) {
          double AW;

          //  system matrix
          b_A[0] = 1.0;
          b_A[6] = dt;
          b_A[12] = 0.0;
          b_A[18] = 0.0;
          b_A[24] = 0.0;
          b_A[30] = 0.0;
          b_A[3] = 0.0;
          b_A[9] = 0.0;
          b_A[15] = 0.0;
          b_A[21] = 1.0;
          b_A[27] = dt;
          b_A[33] = 0.0;

          //  system noise cov
          BW = 0.0625 * (0.33333333333333331 * rt_powd_snf(dt, 3.0));
          Q[0] = BW;
          AW = 0.0625 * (0.5 * (dt * dt));
          Q[6] = AW;
          Q[12] = 0.0;
          Q[18] = 0.0;
          Q[24] = 0.0;
          Q[30] = 0.0;
          Q[1] = AW;
          Q[7] = 0.0625 * dt;
          Q[13] = 0.0;
          Q[19] = 0.0;
          Q[25] = 0.0;
          Q[31] = 0.0;
          Q[3] = 0.0;
          Q[9] = 0.0;
          Q[15] = 0.0;
          Q[21] = BW;
          Q[27] = AW;
          Q[33] = 0.0;
          Q[4] = 0.0;
          Q[10] = 0.0;
          Q[16] = 0.0;
          Q[22] = AW;
          Q[28] = 0.0625 * dt;
          Q[34] = 0.0;
          for (i = 0; i < 6; i++) {
            b_A[6 * i + 1] = iv3[i];
            b_A_tmp = 6 * i + 2;
            b_A[b_A_tmp] = 0.0;
            b_A[6 * i + 4] = iv4[i];
            c_A_tmp = 6 * i + 5;
            b_A[c_A_tmp] = 0.0;
            Q[b_A_tmp] = 0.0;
            Q[c_A_tmp] = 0.0;
          }
        } else {
          double AW;

          //  system matrix
          b_A[0] = 1.0;
          b_A[6] = dt;
          b_A[12] = 0.0;
          b_A[18] = 0.0;
          b_A[24] = 0.0;
          b_A[30] = 0.0;
          b_A[3] = 0.0;
          b_A[9] = 0.0;
          b_A[15] = 0.0;
          b_A[21] = 1.0;
          b_A[27] = dt;
          b_A[33] = 0.0;

          //  system noise cov
          BW = 0.25 * (0.33333333333333331 * rt_powd_snf(dt, 3.0));
          Q[0] = BW;
          AW = 0.25 * (0.5 * (dt * dt));
          Q[6] = AW;
          Q[12] = 0.0;
          Q[18] = 0.0;
          Q[24] = 0.0;
          Q[30] = 0.0;
          Q[1] = AW;
          Q[7] = 0.25 * dt;
          Q[13] = 0.0;
          Q[19] = 0.0;
          Q[25] = 0.0;
          Q[31] = 0.0;
          Q[3] = 0.0;
          Q[9] = 0.0;
          Q[15] = 0.0;
          Q[21] = BW;
          Q[27] = AW;
          Q[33] = 0.0;
          Q[4] = 0.0;
          Q[10] = 0.0;
          Q[16] = 0.0;
          Q[22] = AW;
          Q[28] = 0.25 * dt;
          Q[34] = 0.0;
          for (i = 0; i < 6; i++) {
            b_A[6 * i + 1] = iv3[i];
            b_A_tmp = 6 * i + 2;
            b_A[b_A_tmp] = 0.0;
            b_A[6 * i + 4] = iv4[i];
            c_A_tmp = 6 * i + 5;
            b_A[c_A_tmp] = 0.0;
            Q[b_A_tmp] = 0.0;
            Q[c_A_tmp] = 0.0;
          }
        }

        //  state-prediction
        for (i = 0; i < 6; i++) {
          BW = 0.0;
          for (i1 = 0; i1 < 6; i1++) {
            BW += b_A[i + 6 * i1] * obj->IMM[imm_idx].x[i1];
          }

          c_A[i] = BW;
        }

        for (i = 0; i < 6; i++) {
          obj->IMM[imm_idx].x[i] = c_A[i];
        }

        //  cov-prediction
        if (dt > 0.0) {
          for (i = 0; i < 6; i++) {
            for (i1 = 0; i1 < 6; i1++) {
              BW = 0.0;
              for (c_A_tmp = 0; c_A_tmp < 6; c_A_tmp++) {
                BW += b_A[i + 6 * c_A_tmp] * obj->IMM[imm_idx].P[c_A_tmp + 6 *
                  i1];
              }

              d_A[i + 6 * i1] = BW;
            }

            for (i1 = 0; i1 < 6; i1++) {
              BW = 0.0;
              for (c_A_tmp = 0; c_A_tmp < 6; c_A_tmp++) {
                BW += d_A[i + 6 * c_A_tmp] * b_A[i1 + 6 * c_A_tmp];
              }

              b_A_tmp = i + 6 * i1;
              A[b_A_tmp] = BW + Q[b_A_tmp];
            }
          }
        }

        std::memcpy(&obj->IMM[imm_idx].P[0], &A[0], 36U * sizeof(double));
      }
    }
  }
}

//
// File trailer for imm_predict.cpp
//
// [EOF]
//
