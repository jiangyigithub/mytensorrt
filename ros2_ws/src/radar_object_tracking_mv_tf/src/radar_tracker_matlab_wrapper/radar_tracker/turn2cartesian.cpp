//
// File: turn2cartesian.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "turn2cartesian.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// Transformation von [dx,dy,h,omega,s, 0]' zu [dx,vx,ax,dy,vy,ay]'
//  siehe Blackman, Popoli, "Modern Tracking Systems", S. 229
// Arguments    : const double x_T[6]
//                const double P_T[36]
//                double x_C[6]
//                double P_C[36]
// Return Type  : void
//
void turn2cartesian(const double x_T[6], const double P_T[36], double x_C[6],
                    double P_C[36])
{
  double vx_tmp;
  double vx;
  double vy_tmp;
  double vy;
  double ax;
  double ay;
  double A_C[36];
  int i;
  static const signed char iv[6] = { 1, 0, 0, 0, 0, 0 };

  static const signed char iv1[6] = { 0, 1, 0, 0, 0, 0 };

  double b_A_C[36];
  vx_tmp = std::cos(x_T[2]);
  vx = x_T[4] * vx_tmp;
  vy_tmp = std::sin(x_T[2]);
  vy = x_T[4] * vy_tmp;
  ax = -x_T[3] * vy;
  ay = x_T[3] * vx;
  A_C[1] = 0.0;
  A_C[7] = 0.0;
  A_C[13] = -vy;
  A_C[19] = 0.0;
  A_C[25] = vx_tmp;
  A_C[31] = 0.0;
  A_C[2] = 0.0;
  A_C[8] = 0.0;
  A_C[14] = -ay;
  A_C[20] = -vy;
  A_C[26] = -x_T[3] * vy_tmp;
  A_C[32] = 0.0;
  for (i = 0; i < 6; i++) {
    A_C[6 * i] = iv[i];
    A_C[6 * i + 3] = iv1[i];
  }

  A_C[4] = 0.0;
  A_C[10] = 0.0;
  A_C[16] = vx;
  A_C[22] = 0.0;
  A_C[28] = vy_tmp;
  A_C[34] = 0.0;
  A_C[5] = 0.0;
  A_C[11] = 0.0;
  A_C[17] = ax;
  A_C[23] = vx;
  A_C[29] = x_T[3] * vx_tmp;
  A_C[35] = 0.0;
  x_C[0] = x_T[0];
  x_C[1] = vx;
  x_C[2] = ax;
  x_C[3] = x_T[1];
  x_C[4] = vy;
  x_C[5] = ay;

  //  Für Varianzen muss die Relativgeschwindigkeit verwendet werden,
  //  da die Varianzen bei schnellen Objekten sonst riesig werden können (Unsicherheit im Heading schlägt durch?) 
  //  Diese Lösung ist numerisch instabil :-(
  //  A_C(5,3) = A_C(5,3) - vx_ego;
  //  A_C(6,4) = A_C(6,4) - vx_ego;
  for (i = 0; i < 6; i++) {
    int i1;
    int i2;
    for (i1 = 0; i1 < 6; i1++) {
      vx_tmp = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        vx_tmp += A_C[i + 6 * i2] * P_T[i2 + 6 * i1];
      }

      b_A_C[i + 6 * i1] = vx_tmp;
    }

    for (i1 = 0; i1 < 6; i1++) {
      vx_tmp = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        vx_tmp += b_A_C[i + 6 * i2] * A_C[i1 + 6 * i2];
      }

      P_C[i + 6 * i1] = vx_tmp;
    }
  }

  //  % Bei Initialisierung können Varianzen der Beschleunigung 0 werden
  //  P_C(3,3) = max(P_C(3,3),1);
  //  P_C(6,6) = max(P_C(6,6),1);
}

//
// File trailer for turn2cartesian.cpp
//
// [EOF]
//
