//
// File: xzgeqp3.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "xzgeqp3.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>
#include <cstring>

// Function Definitions

//
// Arguments    : double A_data[]
//                double tau_data[]
//                int jpvt[3]
// Return Type  : void
//
void qrpf(double A_data[], double tau_data[], int jpvt[3])
{
  int j;
  double work[3];
  int pvt;
  double smax;
  double scale;
  int kend;
  int k;
  double absxk;
  double vn1[3];
  double vn2[3];
  for (j = 0; j < 3; j++) {
    work[j] = 0.0;
    pvt = j * 5;
    smax = 0.0;
    scale = 3.3121686421112381E-170;
    kend = pvt + 5;
    for (k = pvt + 1; k <= kend; k++) {
      absxk = std::abs(A_data[k - 1]);
      if (absxk > scale) {
        double t;
        t = scale / absxk;
        smax = smax * t * t + 1.0;
        scale = absxk;
      } else {
        double t;
        t = absxk / scale;
        smax += t * t;
      }
    }

    smax = scale * std::sqrt(smax);
    vn1[j] = smax;
    vn2[j] = smax;
  }

  for (int i = 0; i < 3; i++) {
    int ip1;
    int ii;
    int ix;
    int b_i;
    ip1 = i + 2;
    ii = i * 5 + i;
    kend = 3 - i;
    pvt = 0;
    if (3 - i > 1) {
      ix = i;
      smax = std::abs(vn1[i]);
      for (k = 2; k <= kend; k++) {
        ix++;
        scale = std::abs(vn1[ix]);
        if (scale > smax) {
          pvt = k - 1;
          smax = scale;
        }
      }
    }

    pvt += i;
    if (pvt != i) {
      ix = pvt * 5;
      kend = i * 5;
      for (k = 0; k < 5; k++) {
        smax = A_data[ix];
        A_data[ix] = A_data[kend];
        A_data[kend] = smax;
        ix++;
        kend++;
      }

      kend = jpvt[pvt];
      jpvt[pvt] = jpvt[i];
      jpvt[i] = kend;
      vn1[pvt] = vn1[i];
      vn2[pvt] = vn2[i];
    }

    absxk = A_data[ii];
    pvt = ii + 2;
    tau_data[i] = 0.0;
    smax = b_xnrm2(4 - i, A_data, ii + 2);
    if (smax != 0.0) {
      scale = rt_hypotd_snf(A_data[ii], smax);
      if (A_data[ii] >= 0.0) {
        scale = -scale;
      }

      if (std::abs(scale) < 1.0020841800044864E-292) {
        kend = -1;
        b_i = (ii - i) + 5;
        do {
          kend++;
          for (k = pvt; k <= b_i; k++) {
            A_data[k - 1] *= 9.9792015476736E+291;
          }

          scale *= 9.9792015476736E+291;
          absxk *= 9.9792015476736E+291;
        } while (!(std::abs(scale) >= 1.0020841800044864E-292));

        scale = rt_hypotd_snf(absxk, b_xnrm2(4 - i, A_data, ii + 2));
        if (absxk >= 0.0) {
          scale = -scale;
        }

        tau_data[i] = (scale - absxk) / scale;
        smax = 1.0 / (absxk - scale);
        for (k = pvt; k <= b_i; k++) {
          A_data[k - 1] *= smax;
        }

        for (k = 0; k <= kend; k++) {
          scale *= 1.0020841800044864E-292;
        }

        absxk = scale;
      } else {
        tau_data[i] = (scale - A_data[ii]) / scale;
        smax = 1.0 / (A_data[ii] - scale);
        b_i = (ii - i) + 5;
        for (k = pvt; k <= b_i; k++) {
          A_data[k - 1] *= smax;
        }

        absxk = scale;
      }
    }

    A_data[ii] = absxk;
    if (i + 1 < 3) {
      int lastv;
      int lastc;
      absxk = A_data[ii];
      A_data[ii] = 1.0;
      pvt = ii + 6;
      if (tau_data[i] != 0.0) {
        bool exitg2;
        lastv = 5 - i;
        kend = (ii - i) + 4;
        while ((lastv > 0) && (A_data[kend] == 0.0)) {
          lastv--;
          kend--;
        }

        lastc = 1 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc + 1 > 0)) {
          int exitg1;
          kend = (ii + lastc * 5) + 5;
          j = kend;
          do {
            exitg1 = 0;
            if (j + 1 <= kend + lastv) {
              if (A_data[j] != 0.0) {
                exitg1 = 1;
              } else {
                j++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = -1;
      }

      if (lastv > 0) {
        int i1;
        if (lastc + 1 != 0) {
          if (0 <= lastc) {
            std::memset(&work[0], 0, (lastc + 1) * sizeof(double));
          }

          kend = 0;
          b_i = (ii + 5 * lastc) + 6;
          for (k = pvt; k <= b_i; k += 5) {
            ix = ii;
            smax = 0.0;
            i1 = (k + lastv) - 1;
            for (j = k; j <= i1; j++) {
              smax += A_data[j - 1] * A_data[ix];
              ix++;
            }

            work[kend] += smax;
            kend++;
          }
        }

        if (!(-tau_data[i] == 0.0)) {
          kend = ii;
          pvt = 0;
          for (j = 0; j <= lastc; j++) {
            if (work[pvt] != 0.0) {
              smax = work[pvt] * -tau_data[i];
              ix = ii;
              b_i = kend + 6;
              i1 = lastv + kend;
              for (k = b_i; k <= i1 + 5; k++) {
                A_data[k - 1] += A_data[ix] * smax;
                ix++;
              }
            }

            pvt++;
            kend += 5;
          }
        }
      }

      A_data[ii] = absxk;
    }

    for (j = ip1; j < 4; j++) {
      kend = i + (j - 1) * 5;
      smax = vn1[j - 1];
      if (smax != 0.0) {
        scale = std::abs(A_data[kend]) / smax;
        scale = 1.0 - scale * scale;
        if (scale < 0.0) {
          scale = 0.0;
        }

        absxk = smax / vn2[j - 1];
        absxk = scale * (absxk * absxk);
        if (absxk <= 1.4901161193847656E-8) {
          smax = b_xnrm2(4 - i, A_data, kend + 2);
          vn1[j - 1] = smax;
          vn2[j - 1] = smax;
        } else {
          vn1[j - 1] = smax * std::sqrt(scale);
        }
      }
    }
  }
}

//
// File trailer for xzgeqp3.cpp
//
// [EOF]
//
