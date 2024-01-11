//
// File: qrsolve.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "qrsolve.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xzgeqp3.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Arguments    : const double A_data[]
//                const int A_size[1]
//                const double B_data[]
//                const int B_size[1]
// Return Type  : double
//
double qrsolve(const double A_data[], const int A_size[1], const double B_data[],
               const int B_size[1])
{
  double Y;
  int A_size_idx_0;
  int minmana;
  double b_A_data[512];
  int rankR;
  double tau_data[1];
  double atmp;
  double wj;
  double beta1;
  double b_B_data[512];
  int k;
  int i;
  A_size_idx_0 = A_size[0];
  minmana = A_size[0];
  if (0 <= minmana - 1) {
    std::memcpy(&b_A_data[0], &A_data[0], minmana * sizeof(double));
  }

  rankR = A_size[0];
  if (A_size[0] < 1) {
    minmana = -1;
  } else {
    minmana = 0;
  }

  if (0 <= minmana) {
    tau_data[0] = 0.0;
  }

  if ((A_size[0] != 0) && (A_size[0] >= 1)) {
    if (1 < A_size[0]) {
      atmp = b_A_data[0];
      tau_data[0] = 0.0;
      wj = xnrm2(A_size[0] - 1, b_A_data);
      if (wj != 0.0) {
        beta1 = rt_hypotd_snf(b_A_data[0], wj);
        if (b_A_data[0] >= 0.0) {
          beta1 = -beta1;
        }

        if (std::abs(beta1) < 1.0020841800044864E-292) {
          minmana = -1;
          do {
            minmana++;
            for (k = 2; k <= rankR; k++) {
              b_A_data[k - 1] *= 9.9792015476736E+291;
            }

            beta1 *= 9.9792015476736E+291;
            atmp *= 9.9792015476736E+291;
          } while (!(std::abs(beta1) >= 1.0020841800044864E-292));

          beta1 = rt_hypotd_snf(atmp, xnrm2(A_size[0] - 1, b_A_data));
          if (atmp >= 0.0) {
            beta1 = -beta1;
          }

          tau_data[0] = (beta1 - atmp) / beta1;
          wj = 1.0 / (atmp - beta1);
          for (k = 2; k <= rankR; k++) {
            b_A_data[k - 1] *= wj;
          }

          for (k = 0; k <= minmana; k++) {
            beta1 *= 1.0020841800044864E-292;
          }

          atmp = beta1;
        } else {
          tau_data[0] = (beta1 - b_A_data[0]) / beta1;
          wj = 1.0 / (b_A_data[0] - beta1);
          for (k = 2; k <= rankR; k++) {
            b_A_data[k - 1] *= wj;
          }

          atmp = beta1;
        }
      }

      b_A_data[0] = atmp;
    } else {
      tau_data[0] = 0.0;
    }
  }

  rankR = 0;
  if (A_size[0] >= 1) {
    wj = std::abs(b_A_data[0]);
    if (!(wj <= 2.2204460492503131E-15 * static_cast<double>(A_size[0]) * wj)) {
      rankR = 1;
    }
  }

  minmana = B_size[0];
  if (0 <= minmana - 1) {
    std::memcpy(&b_B_data[0], &B_data[0], minmana * sizeof(double));
  }

  Y = 0.0;
  if (A_size[0] < 1) {
    minmana = -1;
  } else {
    minmana = 0;
  }

  for (k = 0; k <= minmana; k++) {
    if (tau_data[0] != 0.0) {
      wj = b_B_data[0];
      for (i = 2; i <= A_size_idx_0; i++) {
        wj += b_A_data[i - 1] * b_B_data[i - 1];
      }

      wj *= tau_data[0];
      if (wj != 0.0) {
        b_B_data[0] -= wj;
        for (i = 2; i <= A_size_idx_0; i++) {
          b_B_data[i - 1] -= b_A_data[i - 1] * wj;
        }
      }
    }
  }

  for (i = 0; i < rankR; i++) {
    Y = b_B_data[0];
  }

  for (k = rankR; k >= 1; k--) {
    Y /= b_A_data[0];
  }

  return Y;
}

//
// File trailer for qrsolve.cpp
//
// [EOF]
//
