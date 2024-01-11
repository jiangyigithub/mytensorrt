//
// File: rand.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "rand.h"
#include "radar_tracker.h"
#include "radar_tracker_data.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"

// Function Declarations
static double eml_rand_mt19937ar(unsigned int b_state[625]);

// Function Definitions

//
// Arguments    : unsigned int b_state[625]
// Return Type  : double
//
static double eml_rand_mt19937ar(unsigned int b_state[625])
{
  double b_r;
  unsigned int u[2];
  unsigned int y;

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  do {
    for (int j = 0; j < 2; j++) {
      unsigned int mti;
      mti = b_state[624] + 1U;
      if (mti >= 625U) {
        int kk;
        for (kk = 0; kk < 227; kk++) {
          y = (b_state[kk] & 2147483648U) | (b_state[kk + 1] & 2147483647U);
          if ((y & 1U) == 0U) {
            y >>= 1U;
          } else {
            y = y >> 1U ^ 2567483615U;
          }

          b_state[kk] = b_state[kk + 397] ^ y;
        }

        for (kk = 0; kk < 396; kk++) {
          y = (b_state[kk + 227] & 2147483648U) | (b_state[kk + 228] &
            2147483647U);
          if ((y & 1U) == 0U) {
            y >>= 1U;
          } else {
            y = y >> 1U ^ 2567483615U;
          }

          b_state[kk + 227] = b_state[kk] ^ y;
        }

        y = (b_state[623] & 2147483648U) | (b_state[0] & 2147483647U);
        if ((y & 1U) == 0U) {
          y >>= 1U;
        } else {
          y = y >> 1U ^ 2567483615U;
        }

        b_state[623] = b_state[396] ^ y;
        mti = 1U;
      }

      y = b_state[static_cast<int>(mti) - 1];
      b_state[624] = mti;
      y ^= y >> 11U;
      y ^= y << 7U & 2636928640U;
      y ^= y << 15U & 4022730752U;
      u[j] = y ^ y >> 18U;
    }

    u[0] >>= 5U;
    u[1] >>= 6U;
    b_r = 1.1102230246251565E-16 * (static_cast<double>(u[0]) * 6.7108864E+7 +
      static_cast<double>(u[1]));
  } while (b_r == 0.0);

  return b_r;
}

//
// Arguments    : double varargin_2
//                double r_data[]
//                int r_size[2]
// Return Type  : void
//
void b_rand(double varargin_2, double r_data[], int r_size[2])
{
  int i;
  r_size[0] = 1;
  i = static_cast<int>(varargin_2);
  r_size[1] = i;
  for (int k = 0; k < i; k++) {
    r_data[k] = eml_rand_mt19937ar(state);
  }
}

//
// Arguments    : void
// Return Type  : double
//
double c_rand()
{
  return eml_rand_mt19937ar(state);
}

//
// File trailer for rand.cpp
//
// [EOF]
//
