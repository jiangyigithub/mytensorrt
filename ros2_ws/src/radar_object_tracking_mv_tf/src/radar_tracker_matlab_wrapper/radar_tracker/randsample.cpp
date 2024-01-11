//
// File: randsample.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "randsample.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Arguments    : double varargin_1
//                double y[10]
// Return Type  : void
//
void randsample(double varargin_1, double y[10])
{
  double n;
  int nsel;
  double rp_data[39];
  int rp_size[2];
  bool selected_data[512];
  int idx_data[39];
  int qEnd;
  int iwork_data[39];
  n = std::floor(varargin_1);
  if (40.0 > n) {
    int b_n;
    int i;
    int b_i;
    b_rand(n, rp_data, rp_size);
    b_n = rp_size[1] + 1;
    nsel = rp_size[1];
    if (0 <= nsel - 1) {
      std::memset(&idx_data[0], 0, nsel * sizeof(int));
    }

    i = rp_size[1] - 1;
    for (nsel = 1; nsel <= i; nsel += 2) {
      if ((rp_data[nsel - 1] <= rp_data[nsel]) || rtIsNaN(rp_data[nsel])) {
        idx_data[nsel - 1] = nsel;
        idx_data[nsel] = nsel + 1;
      } else {
        idx_data[nsel - 1] = nsel + 1;
        idx_data[nsel] = nsel;
      }
    }

    if ((rp_size[1] & 1) != 0) {
      idx_data[rp_size[1] - 1] = rp_size[1];
    }

    b_i = 2;
    while (b_i < b_n - 1) {
      int i2;
      int j;
      i2 = b_i << 1;
      j = 1;
      for (int pEnd = b_i + 1; pEnd < b_n; pEnd = qEnd + b_i) {
        int p;
        int q;
        int kEnd;
        p = j;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > b_n) {
          qEnd = b_n;
        }

        nsel = 0;
        kEnd = qEnd - j;
        while (nsel + 1 <= kEnd) {
          double b_r;
          b_r = rp_data[idx_data[q] - 1];
          i = idx_data[p - 1];
          if ((rp_data[i - 1] <= b_r) || rtIsNaN(b_r)) {
            iwork_data[nsel] = i;
            p++;
            if (p == pEnd) {
              while (q + 1 < qEnd) {
                nsel++;
                iwork_data[nsel] = idx_data[q];
                q++;
              }
            }
          } else {
            iwork_data[nsel] = idx_data[q];
            q++;
            if (q + 1 == qEnd) {
              while (p < pEnd) {
                nsel++;
                iwork_data[nsel] = idx_data[p - 1];
                p++;
              }
            }
          }

          nsel++;
        }

        for (nsel = 0; nsel < kEnd; nsel++) {
          idx_data[(j + nsel) - 1] = iwork_data[nsel];
        }

        j = qEnd;
      }

      b_i = i2;
    }

    nsel = rp_size[0] * rp_size[1];
    for (i = 0; i < nsel; i++) {
      rp_data[i] = idx_data[i];
    }

    std::memcpy(&y[0], &rp_data[0], 10U * sizeof(double));
  } else {
    nsel = static_cast<int>(n);
    if (0 <= nsel - 1) {
      std::memset(&selected_data[0], 0, nsel * sizeof(bool));
    }

    nsel = 0;
    while (nsel < 10) {
      int i;
      double b_r;
      b_r = c_rand();
      b_r = std::floor(b_r * n);
      i = static_cast<int>(b_r + 1.0) - 1;
      if (!selected_data[i]) {
        selected_data[i] = true;
        nsel++;
        y[nsel - 1] = b_r + 1.0;
      }
    }
  }
}

//
// File trailer for randsample.cpp
//
// [EOF]
//
