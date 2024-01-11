//
// File: mgmt_pex.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "mgmt_pex.h"
#include "det.h"
#include "radar_tracker.h"
#include "radar_tracker_init.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

#include <iostream>
// Function Definitions

//
// update existance probability
// Arguments    : OBJECT_STRUCT *obj
//                const e_struct_T asso_list[20]
// Return Type  : void
//
void mgmt_pex(OBJECT_STRUCT *obj, const e_struct_T asso_list[20])
{
  double p_other;
  double LLR;
  double S[9];
  
  std::cout<<"get into mgmt_pex"<<obj->measureable<<std::endl; //lnl
  //  for smoothing pexist
  if (obj->measureable) {
    double measurement_volume;
    double range;
    double num_targets_observed;
    double expected_num_targets;
    double p_bike;
    double PDH1_tmp;
    double PDH0;
    int i;
    measurement_volume = obj->asso.range_interval * obj->asso.vel_interval *
      obj->asso.ang_interval / 0.4 / 0.2 / 0.13962634015954636;

    //  range, Doppler and azimuth separability
    if ((1.0 > measurement_volume) || rtIsNaN(measurement_volume)) {
      measurement_volume = 1.0;
    }

    range = std::sqrt(obj->x[0] * obj->x[0] + obj->x[3] * obj->x[3]);
    num_targets_observed = obj->meas_angles;
    if (!(num_targets_observed < 20.0)) {
      num_targets_observed = 20.0;
    }

    //  get PDH1, PDH0
    expected_num_targets = obj->P_obj_type[0];
    p_bike = obj->P_obj_type[1];

    //  object and non-detection probability of an object at a certain range.
    //  PDH0: Probability, that an object does not exist, although detected (false alarm probability) 
    //  PDH1: Probability, that an object does exist, if detected (detection probability) 
    //  PDH1 heuristic
    //  PDH car
    //  meter, distance where detection and non-detection are equally likely, gen5 corner 
    //  PDH1 reaches 0.8 at expected_range
    //  PDH bike
    //  meter, distance where detection and non-detection are equally likely, gen5 corner 
    //  PDH unknown
    //  meter, distance where detection and non-detection are equally likely, gen5 corner 
    p_other = obj->P_obj_type[0] + obj->P_obj_type[1];
    if (p_other > 1.0) {
      expected_num_targets = obj->P_obj_type[0] / p_other;
      p_bike = obj->P_obj_type[1] / p_other;
      p_other = 0.0;
    } else {
      p_other = (1.0 - obj->P_obj_type[1]) - obj->P_obj_type[0];
    }

    PDH1_tmp = -range / 50.0;
    p_bike = (0.99 * std::exp(-range / 80.0 / 5.0) * expected_num_targets + 0.99
              * std::exp(-range / 30.0 / 5.0) * p_bike) + 0.99 * std::exp
      (PDH1_tmp / 5.0) * p_other;

    //  PDH0 calculation
    //  def:
    //  calc
    PDH0 = 0.005 * measurement_volume;
    p_other = p_bike * 0.1;
    if ((!(PDH0 < p_other)) && (!rtIsNaN(p_other))) {
      PDH0 = p_other;
    }

    //  convert probability to log likelihood ratio
    LLR = std::log(obj->pexist / (1.0 - obj->pexist));
    i = static_cast<int>(num_targets_observed);
    for (int asso_cnt = 0; asso_cnt < i; asso_cnt++) {
      // IMM_MERGE_KINEMATIC_FIT merges the statistical distance^2 and the
      // corresponding covariance of multiple IMMs
      std::memset(&S[0], 0, 9U * sizeof(double));
      p_other = 0.0;
      for (int imm_num = 0; imm_num < 3; imm_num++) {
        for (int i1 = 0; i1 < 3; i1++) {
          int S_tmp;
          int b_S_tmp;
          S_tmp = 3 * i1 + 9 * imm_num;
          S[3 * i1] += obj->IMM[imm_num].mu * asso_list[asso_cnt].S[S_tmp];
          b_S_tmp = 3 * i1 + 1;
          S[b_S_tmp] += obj->IMM[imm_num].mu * asso_list[asso_cnt].S[S_tmp + 1];
          b_S_tmp = 3 * i1 + 2;
          S[b_S_tmp] += obj->IMM[imm_num].mu * asso_list[asso_cnt].S[S_tmp + 2];

          //lnl, brake to stop 
          // std::cout<<"0_mgmt_pex, imm_num, i1, IMM[imm_num].mu, asso_list[asso_cnt].S[S_tmp], S[3 * i1] "<<imm_num<<" i1: "<<i1<<" IMM[imm_num].mu: "<<obj->IMM[imm_num].mu<<" asso_list[asso_cnt].S[S_tmp]: "<<asso_list[asso_cnt].S[S_tmp]<<" sum S[3 * i1]: "<<S[3 * i1]<<std::endl;
        }

        p_other += obj->IMM[imm_num].mu * asso_list[asso_cnt].d2[imm_num];
      }

      //  likelihood ratio update for kinematic fit
      //  blackman/popoli Eq. 6.6
      //  likelihood ratio update for signal part
      //  blackman/popoli Eq. 6.10a
      p_other = std::log(measurement_volume * std::exp(-p_other / 2.0) /
                         (2.5066282746310002 * std::sqrt(det(S)))) + std::log
        (p_bike / PDH0);
      if (!(p_other > 0.1)) {
        p_other = 0.1;
      }

      //  ensure small incremant
      //  sanity check
      if (!rtIsInf(p_other)) {
        LLR += 0.1 * p_other;
      }
    }

    expected_num_targets = obj->length * obj->width * std::exp(PDH1_tmp);

    //  heuristic
    if (range > 50.0) {
      if (!(expected_num_targets > 1.0)) {
        expected_num_targets = 1.0;
      }
    } else if (range > 20.0) {
      if (!(expected_num_targets > 2.0)) {
        expected_num_targets = 2.0;
      }
    } else {
      if (!(expected_num_targets > 3.0)) {
        expected_num_targets = 3.0;
      }
    }

    if (expected_num_targets > num_targets_observed) {
      //  likelihood update for non-observed
      //  blackman/popoli Eq 6.10b
      p_other = std::log((1.0 - p_bike) / (1.0 - PDH0));
      if (!(p_other < -0.1)) {
        p_other = -0.1;
      }

      //  ensure small decrement
      LLR += 0.1 * (expected_num_targets - num_targets_observed) * p_other;
    }

    //  convert log-likelihood ratio to probability
    p_other = std::exp(LLR);
    obj->pexist = p_other / (p_other + 1.0);

    // std::cout<<"pexist in mgmt_pex:"<<obj->pexist<<"p other:"<<p_other<<std::endl;  //lnl

    //  limit pexist
    if (!(obj->pexist < 0.995)) {
      obj->pexist = 0.995;
    }
  }
}

//
// File trailer for mgmt_pex.cpp
//
// [EOF]
//
