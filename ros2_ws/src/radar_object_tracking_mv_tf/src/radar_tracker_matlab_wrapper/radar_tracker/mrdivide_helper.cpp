//
// File: mrdivide_helper.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "mrdivide_helper.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Arguments    : const double A[18]
//                const double B[9]
//                double Y[18]
// Return Type  : void
//
void b_mrdiv(const double A[18], const double B[9], double Y[18])
{
  double b_A[9];
  int r1;
  int r2;
  int r3;
  double maxval;
  double a21;
  int rtemp;
  std::memcpy(&b_A[0], &B[0], 9U * sizeof(double));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(B[0]);
  a21 = std::abs(B[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(B[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = B[r2] / B[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (std::abs(b_A[r3 + 3]) > std::abs(b_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  for (rtemp = 0; rtemp < 6; rtemp++) {
    int Y_tmp;
    int b_Y_tmp;
    int c_Y_tmp;
    Y_tmp = rtemp + 6 * r1;
    Y[Y_tmp] = A[rtemp] / b_A[r1];
    b_Y_tmp = rtemp + 6 * r2;
    Y[b_Y_tmp] = A[rtemp + 6] - Y[Y_tmp] * b_A[r1 + 3];
    c_Y_tmp = rtemp + 6 * r3;
    Y[c_Y_tmp] = A[rtemp + 12] - Y[Y_tmp] * b_A[r1 + 6];
    Y[b_Y_tmp] /= b_A[r2 + 3];
    Y[c_Y_tmp] -= Y[b_Y_tmp] * b_A[r2 + 6];
    Y[c_Y_tmp] /= b_A[r3 + 6];
    Y[b_Y_tmp] -= Y[c_Y_tmp] * b_A[r3 + 3];
    Y[Y_tmp] -= Y[c_Y_tmp] * b_A[r3];
    Y[Y_tmp] -= Y[b_Y_tmp] * b_A[r2];
  }
}

//
// Arguments    : const double A[10]
//                const double B[10]
// Return Type  : double
//
double mrdiv(const double A[10], const double B[10])
{
  double Y;
  double b_A[10];
  double b_B[10];
  double atmp;
  double tau;
  double xnorm;
  double scale;
  int k;
  double absxk;
  double t;
  int knt;
  std::memcpy(&b_A[0], &B[0], 10U * sizeof(double));
  std::memcpy(&b_B[0], &A[0], 10U * sizeof(double));
  atmp = b_A[0];
  tau = 0.0;
  xnorm = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 9; k++) {
    absxk = std::abs(b_A[k + 1]);
    if (absxk > scale) {
      t = scale / absxk;
      xnorm = xnorm * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      xnorm += t * t;
    }
  }

  xnorm = scale * std::sqrt(xnorm);
  if (xnorm != 0.0) {
    xnorm = rt_hypotd_snf(b_A[0], xnorm);
    if (b_A[0] >= 0.0) {
      xnorm = -xnorm;
    }

    if (std::abs(xnorm) < 1.0020841800044864E-292) {
      knt = -1;
      do {
        knt++;
        for (k = 0; k < 9; k++) {
          b_A[k + 1] *= 9.9792015476736E+291;
        }

        xnorm *= 9.9792015476736E+291;
        atmp *= 9.9792015476736E+291;
      } while (!(std::abs(xnorm) >= 1.0020841800044864E-292));

      xnorm = 0.0;
      scale = 3.3121686421112381E-170;
      for (k = 0; k < 9; k++) {
        absxk = std::abs(b_A[k + 1]);
        if (absxk > scale) {
          t = scale / absxk;
          xnorm = xnorm * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          xnorm += t * t;
        }
      }

      xnorm = rt_hypotd_snf(atmp, scale * std::sqrt(xnorm));
      if (atmp >= 0.0) {
        xnorm = -xnorm;
      }

      tau = (xnorm - atmp) / xnorm;
      scale = 1.0 / (atmp - xnorm);
      for (k = 0; k < 9; k++) {
        b_A[k + 1] *= scale;
      }

      for (k = 0; k <= knt; k++) {
        xnorm *= 1.0020841800044864E-292;
      }

      atmp = xnorm;
    } else {
      tau = (xnorm - b_A[0]) / xnorm;
      scale = 1.0 / (b_A[0] - xnorm);
      for (k = 0; k < 9; k++) {
        b_A[k + 1] *= scale;
      }

      atmp = xnorm;
    }
  }

  b_A[0] = atmp;
  knt = 0;
  scale = std::abs(atmp);
  if (!(scale <= 2.2204460492503131E-14 * scale)) {
    knt = 1;
  }

  Y = 0.0;
  if (tau != 0.0) {
    scale = A[0];
    for (k = 0; k < 9; k++) {
      scale += b_A[k + 1] * A[k + 1];
    }

    scale *= tau;
    if (scale != 0.0) {
      b_B[0] = A[0] - scale;
      for (k = 0; k < 9; k++) {
        b_B[k + 1] -= b_A[k + 1] * scale;
      }
    }
  }

  for (k = 0; k < knt; k++) {
    Y = b_B[0];
  }

  for (k = knt; k >= 1; k--) {
    Y /= atmp;
  }

  return Y;
}

//
// File trailer for mrdivide_helper.cpp
//
// [EOF]
//
