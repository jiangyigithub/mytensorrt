//
// File: cartesian2turn.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "cartesian2turn.h"
#include "mgmt_init_track.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// Transform [dx,vx,ax,dy,vy,ay]' to [dx,dy,h,omega,s, 0]'
//  see Blackman, Popoli, "Modern Tracking Systems", S. 229
// Arguments    : const double x_C[6]
//                const double P_C[36]
//                double x_T[6]
//                double P_T[36]
// Return Type  : void
//
void cartesian2turn(const double x_C[6], const double P_C[36], double x_T[6],
                    double P_T[36])
{
  double h;
  double s_tmp;
  double b_s_tmp;
  double s;
  double A_T[36];
  static const signed char iv[6] = { 1, 0, 0, 0, 0, 0 };

  static const signed char iv1[6] = { 0, 0, 0, 1, 0, 0 };

  static const double dv[6] = { 0.0, 0.0, 2.4674011002723395, 0.0, 0.0, 0.0 };

  double b_A_T[36];
  h = rt_atan2d_snf(x_C[4], x_C[1]);
  s_tmp = x_C[4] * x_C[4];
  b_s_tmp = x_C[1] * x_C[1];
  s = std::sqrt(b_s_tmp + s_tmp);
  if (s > 1.0) {
    double a32_tmp;
    double a32;
    double a35;
    double A_T_tmp;
    int i;
    a32_tmp = s * s;
    a32 = -x_C[4] / a32_tmp;
    a35 = x_C[1] / a32_tmp;
    A_T[2] = 0.0;
    A_T[8] = a32;
    A_T[14] = 0.0;
    A_T[20] = 0.0;
    A_T[26] = a35;
    A_T[32] = 0.0;
    A_T[3] = 0.0;
    s_tmp -= b_s_tmp;
    b_s_tmp = 2.0 * x_C[1] * x_C[4];
    A_T_tmp = rt_powd_snf(s, 4.0);
    A_T[9] = (x_C[5] * s_tmp + b_s_tmp * x_C[2]) / A_T_tmp;
    A_T[15] = a32;
    A_T[21] = 0.0;
    A_T[27] = (x_C[2] * s_tmp - b_s_tmp * x_C[5]) / A_T_tmp;
    A_T[33] = a35;
    A_T[4] = 0.0;
    A_T[10] = x_C[1] / s;
    A_T[16] = 0.0;
    A_T[22] = 0.0;
    A_T[28] = x_C[4] / s;
    A_T[34] = 0.0;
    for (i = 0; i < 6; i++) {
      A_T[6 * i] = iv[i];
      A_T[6 * i + 1] = iv1[i];
      A_T[6 * i + 5] = 0.0;
    }

    x_T[0] = x_C[0];
    x_T[1] = x_C[3];
    x_T[2] = h;
    x_T[3] = (x_C[1] * x_C[5] - x_C[4] * x_C[2]) / a32_tmp;
    x_T[4] = s;
    x_T[5] = 0.0;
    for (i = 0; i < 6; i++) {
      int i1;
      int i2;
      for (i1 = 0; i1 < 6; i1++) {
        s_tmp = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          s_tmp += A_T[i + 6 * i2] * P_C[i2 + 6 * i1];
        }

        b_A_T[i + 6 * i1] = s_tmp;
      }

      for (i1 = 0; i1 < 6; i1++) {
        s_tmp = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          s_tmp += b_A_T[i + 6 * i2] * A_T[i1 + 6 * i2];
        }

        P_T[i + 6 * i1] = s_tmp;
      }
    }
  } else {
    x_T[0] = x_C[0];
    x_T[1] = x_C[3];
    x_T[2] = h;
    x_T[3] = 0.0;
    x_T[4] = s;
    x_T[5] = 0.0;
    P_T[0] = P_C[0];
    P_T[6] = 0.0;
    P_T[12] = 0.0;
    P_T[18] = 0.0;
    P_T[24] = 0.0;
    P_T[30] = 0.0;
    P_T[1] = 0.0;
    P_T[7] = P_C[21];
    P_T[13] = 0.0;
    P_T[19] = 0.0;
    P_T[25] = 0.0;
    P_T[31] = 0.0;
    P_T[4] = 0.0;
    P_T[10] = 0.0;
    P_T[16] = 0.0;
    P_T[22] = 0.0;
    if ((P_C[7] > P_C[28]) || rtIsNaN(P_C[28])) {
      P_T[28] = P_C[7];
    } else {
      P_T[28] = P_C[28];
    }

    P_T[34] = 0.0;
    for (int i = 0; i < 6; i++) {
      P_T[6 * i + 2] = dv[i];
      P_T[6 * i + 3] = iv1[i];
      P_T[6 * i + 5] = 0.0;
    }
  }
}

//
// File trailer for cartesian2turn.cpp
//
// [EOF]
//
