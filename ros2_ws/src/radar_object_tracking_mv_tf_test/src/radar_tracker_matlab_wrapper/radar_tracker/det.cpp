//
// File: det.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "det.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Arguments    : const double x[9]
// Return Type  : double
//
double det(const double x[9])
{
  double y;
  double b_x[9];
  signed char ipiv[3];
  bool isodd;
  int iy;
  int jA;
  int ix;
  std::memcpy(&b_x[0], &x[0], 9U * sizeof(double));
  ipiv[0] = 1;
  ipiv[1] = 2;
  for (int j = 0; j < 2; j++) {
    int mmj_tmp;
    int b_tmp;
    int jp1j;
    double smax;
    int k;
    int i;
    mmj_tmp = 1 - j;
    b_tmp = j << 2;
    jp1j = b_tmp + 2;
    iy = 3 - j;
    jA = 0;
    ix = b_tmp;
    smax = std::abs(b_x[b_tmp]);
    for (k = 2; k <= iy; k++) {
      double s;
      ix++;
      s = std::abs(b_x[ix]);
      if (s > smax) {
        jA = k - 1;
        smax = s;
      }
    }

    if (b_x[b_tmp + jA] != 0.0) {
      if (jA != 0) {
        iy = j + jA;
        ipiv[j] = static_cast<signed char>(iy + 1);
        smax = b_x[j];
        b_x[j] = b_x[iy];
        b_x[iy] = smax;
        ix = j + 3;
        iy += 3;
        smax = b_x[ix];
        b_x[ix] = b_x[iy];
        b_x[iy] = smax;
        ix += 3;
        iy += 3;
        smax = b_x[ix];
        b_x[ix] = b_x[iy];
        b_x[iy] = smax;
      }

      i = (b_tmp - j) + 3;
      for (iy = jp1j; iy <= i; iy++) {
        b_x[iy - 1] /= b_x[b_tmp];
      }
    }

    iy = b_tmp + 3;
    jA = b_tmp;
    for (k = 0; k <= mmj_tmp; k++) {
      smax = b_x[iy];
      if (b_x[iy] != 0.0) {
        ix = b_tmp + 1;
        i = jA + 5;
        jp1j = (jA - j) + 6;
        for (int ijA = i; ijA <= jp1j; ijA++) {
          b_x[ijA - 1] += b_x[ix] * -smax;
          ix++;
        }
      }

      iy += 3;
      jA += 3;
    }
  }

  isodd = (ipiv[0] > 1);
  y = b_x[0] * b_x[4] * b_x[8];
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }

  if (isodd) {
    y = -y;
  }

  return y;
}

//
// File trailer for det.cpp
//
// [EOF]
//
