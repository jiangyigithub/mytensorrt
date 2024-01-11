//
// File: polyfit.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "polyfit.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"

// Function Definitions

//
// Arguments    : const double x_data[]
//                const double y_data[]
//                double p[3]
// Return Type  : void
//
void polyfit(const double x_data[], const double y_data[], double p[3])
{
  int k;
  int jpvt[3];
  double V_data[15];
  double tau_data[3];
  double B_data[5];
  int j;
  double wj;
  int i;
  for (k = 0; k < 5; k++) {
    V_data[k + 10] = 1.0;
    V_data[k + 5] = x_data[k];
    V_data[k] = x_data[k] * x_data[k];
    B_data[k] = y_data[k];
  }

  jpvt[0] = 1;
  tau_data[0] = 0.0;
  jpvt[1] = 2;
  tau_data[1] = 0.0;
  jpvt[2] = 3;
  tau_data[2] = 0.0;
  qrpf(V_data, tau_data, jpvt);
  for (j = 0; j < 3; j++) {
    p[j] = 0.0;
    if (tau_data[j] != 0.0) {
      wj = B_data[j];
      k = j + 2;
      for (i = k; i < 6; i++) {
        wj += V_data[(i + 5 * j) - 1] * B_data[i - 1];
      }

      wj *= tau_data[j];
      if (wj != 0.0) {
        B_data[j] -= wj;
        k = j + 2;
        for (i = k; i < 6; i++) {
          B_data[i - 1] -= V_data[(i + 5 * j) - 1] * wj;
        }
      }
    }
  }

  p[jpvt[0] - 1] = B_data[0];
  p[jpvt[1] - 1] = B_data[1];
  p[jpvt[2] - 1] = B_data[2];
  for (j = 2; j >= 0; j--) {
    p[jpvt[j] - 1] /= V_data[j + 5 * j];
    for (i = 0; i < j; i++) {
      p[jpvt[i] - 1] -= p[jpvt[j] - 1] * V_data[i + 5 * j];
    }
  }
}

//
// File trailer for polyfit.cpp
//
// [EOF]
//
