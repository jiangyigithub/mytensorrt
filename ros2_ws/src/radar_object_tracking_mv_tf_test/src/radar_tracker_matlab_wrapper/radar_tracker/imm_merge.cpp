//
// File: imm_merge.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "imm_merge.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

//lnl, brake to stop
#include <iostream>

// Function Definitions

//
// IMM Merge: estimate joint state vector for data association
//
//  Input:
//  IMM:
//    IMM(i).x: state vector of model i
//    IMM(i).P: covariance of model i
//    IMM(i).mu: Probability for model i
//
//  Output:
//  x: Zusammengef�hrter Zustand
//  P: Zusammengef�hrte Kovarianz
// Arguments    : OBJECT_STRUCT *obj
// Return Type  : void
//
void imm_merge(OBJECT_STRUCT *obj)
{
  struct2_T IMM[3];
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

  int b_i;
  double x[6];
  double b_A_C[36];

  //  Shortcut for IMM-structure
  std::memcpy(&IMM[0], &obj->IMM[0], 3U * sizeof(struct2_T));

  //  transform model 1 from constant-turn-rate to Cartesian
  //  Transformation von [dx,dy,h,omega,s, 0]' zu [dx,vx,ax,dy,vy,ay]'
  //  siehe Blackman, Popoli, "Modern Tracking Systems", S. 229
  vx_tmp = std::cos(obj->IMM[0].x[2]);
  vx = obj->IMM[0].x[4] * vx_tmp;
  vy_tmp = std::sin(obj->IMM[0].x[2]);
  vy = obj->IMM[0].x[4] * vy_tmp;
  ax = -obj->IMM[0].x[3] * vy;
  ay = obj->IMM[0].x[3] * vx;
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
  A_C[26] = -obj->IMM[0].x[3] * vy_tmp;
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
  A_C[29] = obj->IMM[0].x[3] * vx_tmp;
  A_C[35] = 0.0;
  IMM[0].x[0] = obj->IMM[0].x[0];
  IMM[0].x[1] = vx;
  IMM[0].x[2] = ax;
  IMM[0].x[3] = obj->IMM[0].x[1];
  IMM[0].x[4] = vy;
  IMM[0].x[5] = ay;

  //  Für Varianzen muss die Relativgeschwindigkeit verwendet werden,
  //  da die Varianzen bei schnellen Objekten sonst riesig werden können (Unsicherheit im Heading schlägt durch?) 
  //  Diese Lösung ist numerisch instabil :-(
  //  A_C(5,3) = A_C(5,3) - vx_ego;
  //  A_C(6,4) = A_C(6,4) - vx_ego;
  //  % Bei Initialisierung können Varianzen der Beschleunigung 0 werden
  //  P_C(3,3) = max(P_C(3,3),1);
  //  P_C(6,6) = max(P_C(6,6),1);
  //  merge state vectors
  for (b_i = 0; b_i < 6; b_i++) {
    int i1;
    for (i = 0; i < 6; i++) {
      vx_tmp = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        vx_tmp += A_C[b_i + 6 * i1] * obj->IMM[0].P[i1 + 6 * i];
      }

      b_A_C[b_i + 6 * i] = vx_tmp;
    }

    for (i = 0; i < 6; i++) {
      vx_tmp = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        vx_tmp += b_A_C[b_i + 6 * i1] * A_C[i + 6 * i1];
      }

      IMM[0].P[b_i + 6 * i] = vx_tmp;
    }

    x[b_i] = 0.0;
  }

  for (b_i = 0; b_i < 3; b_i++) {
    for (i = 0; i < 6; i++) {
      x[i] += IMM[b_i].mu * IMM[b_i].x[i];

      //lnl, brake to stop 
      // std::cout<<"0_imm_merge, b_i, i, IMM[b_i].mu, IMM[b_i].x[i], x[i]: "<<b_i<<" "<<i<<" IMM[b_i].mu: "<<IMM[b_i].mu<<" "<<IMM[b_i].x[i]<<" sum: "<<x[i]<<std::endl;
    }
  }

  //  merge covariances
  std::memset(&A_C[0], 0, 36U * sizeof(double));
  for (b_i = 0; b_i < 3; b_i++) {
    for (i = 0; i < 36; i++) {
      A_C[i] += IMM[b_i].mu * IMM[b_i].P[i];
    }
  }

  //  set output
  std::memcpy(&obj->P[0], &A_C[0], 36U * sizeof(double));
  for (b_i = 0; b_i < 6; b_i++) {
    obj->x[b_i] = x[b_i];
  }
}

//
// File trailer for imm_merge.cpp
//
// [EOF]
//
