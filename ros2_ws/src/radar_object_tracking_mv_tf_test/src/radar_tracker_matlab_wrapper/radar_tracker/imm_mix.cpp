//
// File: imm_mix.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "imm_mix.h"
#include "cartesian2turn.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include "turn2cartesian.h"
#include <cstring>

//lnl, brake to stop issue
#include <iostream>

// Type Definitions
struct d_struct_T
{
  double Dx_ij[6];
  double DP_ij[36];
};

// Function Definitions

//
// IMM Mixer
//
//  Input:
//  obj.IMM:
// Arguments    : OBJECT_STRUCT *obj
// Return Type  : void
//
void imm_mix(OBJECT_STRUCT *obj)
{
  struct2_T IMM[3];
  int i;
  struct2_T s;
  int j;
  struct2_T IMM_out[3];
  double mu_ij;
  static const double P_ij[9] = { 0.95, 0.01, 0.025, 0.025, 0.98, 0.025, 0.025,
    0.01, 0.95 };

  int b_i;
  d_struct_T b_s;
  int mu_ij_tmp;
  d_struct_T temp[9];
  double b_mu_ij[9];
  double b_IMM_out[36];
  double c_IMM_out[6];

  //    IMM(i).x: state vector of model i
  //    IMM(i).P: covariance of model i
  //    IMM(i).mu: probability for model i
  //
  //  Output:
  //  obj.IMM: Gemischte Zustands-Vektoren und Kovarianzen
  //    IMM(i).x: IMM-mixed state vector of model i
  //    IMM(i).P: IMM-mixed covariance of model i
  //    IMM(i).mu: probability for model i
  //
  //  P_ij: IMM state transition probabilities (row sum == 1)
  //  1: low-dyn CT  (Prior State)
  //  2: high-dyn
  //  3: low-dyn CA
  //   1       2    3       : new state
  //  Shortcut on IMM-structure
  std::memcpy(&IMM[0], &obj->IMM[0], 3U * sizeof(struct2_T));

  //  transform model 1 from constant-turm-rate to Cartesian
  turn2cartesian(obj->IMM[0].x, obj->IMM[0].P, IMM[0].x, IMM[0].P);

  //  initialize output structure
  for (i = 0; i < 6; i++) {
    s.x[i] = 0.0;
  }

  std::memset(&s.P[0], 0, 36U * sizeof(double));
  s.mu = 0.33333333333333331;
  s.pdf_max = 0.0;

  //  --- apply state transition probabilities --------------------------
  //  calculate posterior probability of models
  //  mu_ij(i,j): conditional probability for model change i->j
  //  --- mix state vectors ---------------------------------------------------
  for (j = 0; j < 3; j++) {
    double d;
    double d1;
    IMM_out[j] = s;
    IMM_out[j].mu = 0.0;

    //  IMM_out.mu = P_ij * IMM.mu
    mu_ij = P_ij[3 * j] * IMM[0].mu;

    //  IMM_out.mu = P_ij * IMM.mu
    b_i = 3 * j + 1;
    d = P_ij[b_i] * IMM[1].mu;

    //  IMM_out.mu = P_ij * IMM.mu
    mu_ij_tmp = 3 * j + 2;
    d1 = P_ij[mu_ij_tmp] * IMM[2].mu;
    IMM_out[j].mu = ((IMM_out[j].mu + mu_ij) + d) + d1;
    b_mu_ij[3 * j] = mu_ij / IMM_out[j].mu;
    b_mu_ij[b_i] = d / IMM_out[j].mu;
    b_mu_ij[mu_ij_tmp] = d1 / IMM_out[j].mu;
    for (b_i = 0; b_i < 6; b_i++) {
      IMM_out[j].x[b_i] = 0.0;
    }

    for (i = 0; i < 3; i++) {
      //  IMM_out.x = mu_ij * IMM.x
      mu_ij = b_mu_ij[i + 3 * j];
      for (b_i = 0; b_i < 6; b_i++) {
        IMM_out[j].x[b_i] += mu_ij * IMM[i].x[b_i];
        
        //lnl, brake to stop issue
        // std::cout<<"0_imm_mix, j, i, b_i, IMM_out[j].x[b_i], mu_ij, IMM[i].xb[b_i]: "<<j<<" "<<i<<" "<<b_i<<" IMM_out[j].x[b_i]: "<<IMM_out[j].x[b_i]<<" mu_ij: "<<mu_ij<<" IMM[i].x[b_i]: "<<IMM[i].x[b_i]<<std::endl;

      }
    }
  }

  // --- mix covariances -------------------------------------------------------- 
  //  calculate covariance increment
  //  update state vector difference from i->j
  //  sample covariance of update (Dx_ij) for all i,j
  for (i = 0; i < 6; i++) {
    b_s.Dx_ij[i] = 0.0;
  }

  std::memset(&b_s.DP_ij[0], 0, 36U * sizeof(double));
  for (b_i = 0; b_i < 9; b_i++) {
    temp[b_i] = b_s;
  }

  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      for (b_i = 0; b_i < 6; b_i++) {
        temp[i + 3 * j].Dx_ij[b_i] = IMM[i].x[b_i] - IMM_out[j].x[b_i];
      }

      b_i = i + 3 * j;
      for (mu_ij_tmp = 0; mu_ij_tmp < 6; mu_ij_tmp++) {
        for (int i2 = 0; i2 < 6; i2++) {
          temp[i + 3 * j].DP_ij[i2 + 6 * mu_ij_tmp] = temp[b_i].Dx_ij[i2] *
            temp[b_i].Dx_ij[mu_ij_tmp];
        }
      }
    }
  }

  //  calculate mixed covariance
  for (j = 0; j < 3; j++) {
    mu_ij = b_mu_ij[3 * j];
    for (b_i = 0; b_i < 36; b_i++) {
      IMM_out[j].P[b_i] = mu_ij * (IMM[0].P[b_i] + temp[3 * j].DP_ij[b_i]);
    }

    //  i = 1
    for (i = 0; i < 2; i++) {
      mu_ij_tmp = (i + 3 * j) + 1;
      for (b_i = 0; b_i < 36; b_i++) {
        IMM_out[j].P[b_i] += b_mu_ij[mu_ij_tmp] * (IMM[i + 1].P[b_i] +
          temp[mu_ij_tmp].DP_ij[b_i]);
      }
    }
  }

  //  transform model 1 from Cartesian to constant turn rate
  for (int i1 = 0; i1 < 6; i1++) {
    c_IMM_out[i1] = IMM_out[0].x[i1];
  }

  std::memcpy(&b_IMM_out[0], &IMM_out[0].P[0], 36U * sizeof(double));
  cartesian2turn(c_IMM_out, b_IMM_out, IMM_out[0].x, IMM_out[0].P);

  //  set output
  std::memcpy(&obj->IMM[0], &IMM_out[0], 3U * sizeof(struct2_T));
}

//
// File trailer for imm_mix.cpp
//
// [EOF]
//
