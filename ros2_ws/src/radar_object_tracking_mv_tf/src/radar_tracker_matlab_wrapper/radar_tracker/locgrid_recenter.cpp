//
// File: locgrid_recenter.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "locgrid_recenter.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Declarations
static double rt_roundd_snf(double u);

// Function Definitions

//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// LOCGRID_RECENTER recenters the objects local gridmap, such that the center
// of gravity of the likelihood is near zero.
// Arguments    : OBJECT_STRUCT *obj
// Return Type  : void
//
void locgrid_recenter(OBJECT_STRUCT *obj)
{
  int j;
  double x_likelihood[21];
  double likelihood[441];
  int k;
  int xoffset;
  double y_shift_local;
  double x[21];
  double y;
  double y_likelihood[21];
  double b_y;
  double x_shift_local;

  //  minimum value of LR grid
  //  convert odds to prob
  for (j = 0; j < 441; j++) {
    likelihood[j] = (obj->grid.LR[j] / (obj->grid.LR[j] + 1.0) - 0.1) / 0.9;
  }

  std::memcpy(&x_likelihood[0], &likelihood[0], 21U * sizeof(double));
  for (k = 0; k < 20; k++) {
    xoffset = (k + 1) * 21;
    for (j = 0; j < 21; j++) {
      x_likelihood[j] += likelihood[xoffset + j];
    }
  }

  for (j = 0; j < 21; j++) {
    xoffset = j * 21;
    y_shift_local = likelihood[xoffset];
    for (k = 0; k < 20; k++) {
      y_shift_local += likelihood[(xoffset + k) + 1];
    }

    y_likelihood[j] = y_shift_local;
    x[j] = x_likelihood[j] * obj->grid.x_cells[j];
  }

  y_shift_local = x[0];
  y = x_likelihood[0];
  for (k = 0; k < 20; k++) {
    y_shift_local += x[k + 1];
    y += x_likelihood[k + 1];
  }

  for (j = 0; j < 21; j++) {
    x[j] = y_likelihood[j] * obj->grid.y_cells[j];
  }

  b_y = x[0];
  x_shift_local = y_likelihood[0];
  for (k = 0; k < 20; k++) {
    b_y += x[k + 1];
    x_shift_local += y_likelihood[k + 1];
  }

  y = rt_roundd_snf(y_shift_local / y / obj->grid.step);
  y_shift_local = rt_roundd_snf(b_y / x_shift_local / obj->grid.step);
  if ((-y != 0.0) || (-y_shift_local != 0.0)) {
    double x_shift_global;

    //  shift grid
    std::memcpy(&likelihood[0], &obj->grid.LR[0], 441U * sizeof(double));
    if (!(-y == 0.0)) {
      for (j = 0; j < 441; j++) {
        obj->grid.LR[j] = 0.1;
      }

      if (-y > 0.0) {
        if (1.0 > 21.0 - (-y)) {
          xoffset = 0;
        } else {
          xoffset = static_cast<int>(21.0 - (-y));
        }

        if (-y + 1.0 > 21.0) {
          j = 1;
        } else {
          j = static_cast<int>(-y + 1.0);
        }

        for (k = 0; k < 21; k++) {
          for (int i = 0; i < xoffset; i++) {
            obj->grid.LR[((j + i) + 21 * k) - 1] = likelihood[i + 21 * k];
          }
        }
      } else {
        if (1.0 - (-y) > 21.0) {
          j = 0;
          k = -1;
        } else {
          j = static_cast<int>(1.0 - (-y)) - 1;
          k = 20;
        }

        xoffset = k - j;
        for (k = 0; k < 21; k++) {
          for (int i = 0; i <= xoffset; i++) {
            obj->grid.LR[i + 21 * k] = likelihood[(j + i) + 21 * k];
          }
        }
      }
    }

    std::memcpy(&likelihood[0], &obj->grid.LR[0], 441U * sizeof(double));
    if (!(-y_shift_local == 0.0)) {
      for (j = 0; j < 441; j++) {
        obj->grid.LR[j] = 0.1;
      }

      if (-y_shift_local > 0.0) {
        if (1.0 > 21.0 - (-y_shift_local)) {
          xoffset = 0;
        } else {
          xoffset = static_cast<int>(21.0 - (-y_shift_local));
        }

        if (-y_shift_local + 1.0 > 21.0) {
          j = 1;
        } else {
          j = static_cast<int>(-y_shift_local + 1.0);
        }

        for (k = 0; k < xoffset; k++) {
          std::memcpy(&obj->grid.LR[(k * 21 + j * 21) + -21], &likelihood[k * 21],
                      21U * sizeof(double));
        }
      } else {
        if (1.0 - (-y_shift_local) > 21.0) {
          j = 0;
          k = -1;
        } else {
          j = static_cast<int>(1.0 - (-y_shift_local)) - 1;
          k = 20;
        }

        xoffset = k - j;
        for (k = 0; k <= xoffset; k++) {
          std::memcpy(&obj->grid.LR[k * 21], &likelihood[k * 21 + j * 21], 21U *
                      sizeof(double));
        }
      }
    }

    //  correct object px, py for shift
    x_shift_local = y * obj->grid.step;
    y_shift_local *= obj->grid.step;
    y = std::sin(-obj->psi);
    b_y = std::cos(-obj->psi);
    x_shift_global = b_y * x_shift_local + y * y_shift_local;
    y_shift_local = -y * x_shift_local + b_y * y_shift_local;
    obj->x[0] += x_shift_global;
    obj->x[3] += y_shift_local;
    obj->IMM[0].x[0] += x_shift_global;
    obj->IMM[0].x[1] += y_shift_local;
    obj->IMM[1].x[0] += x_shift_global;
    obj->IMM[1].x[3] += y_shift_local;
    obj->IMM[2].x[0] += x_shift_global;
    obj->IMM[2].x[3] += y_shift_local;
  }
}

//
// File trailer for locgrid_recenter.cpp
//
// [EOF]
//
