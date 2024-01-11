//
// File: radar_tracker.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 05-Aug-2020 12:18:40
//

// Include Files
#include "radar_tracker.h"
#include "asso_meas2track.h"
#include "asso_quick_reject.h"
#include "get_clean_obj.h"
#include "imm_merge.h"
#include "imm_mix.h"
#include "imm_output.h"
#include "imm_predict.h"
#include "immpdaf_mup.h"
#include "locgrid_pre.h"
#include "locgrid_recenter.h"
#include "mgmt_init_track.h"
#include "mgmt_pex.h"
#include "preproc_egomotion_transform.h"
#include "preproc_flagsinit.h"
#include "preproc_plausi.h"
#include "radar_tracker_data.h"
#include "radar_tracker_init.h"
#include "radar_tracker_initialize.h"
#include "radar_tracker_rtwutil.h"
#include "rt_nonfinite.h"
#include "update_derived_values.h"
#include <cmath>
#include <cstring>

//lnl, debugging
#ifndef tracker_initial
#define tracker_initial false
#endif
// #include <iostream>
#include <iostream> //lnl
// Function Definitions

//
// tracking loop with 6D Kalman filter
//
//  x: state vector [dx vx ax dy vy ay]';
//  P: covariance matrix of x
//  y: measurement vector [dr vr alpha]';
//  R: covariance matrix of y
// Arguments    : OBJECT_STRUCT obj_list[50]
//                MEASLIST_STRUCT *meas_list
//                double dt
// Return Type  : void
//
void radar_tracker(OBJECT_STRUCT obj_list[50], MEASLIST_STRUCT *meas_list,
                   double dt)
{
  int obj_idx;
  int obj_idx2;
  static e_struct_T asso_list[1000];
  bool meas_reject[512];
  int vxvy_absGate;
  double refl_y_no_offset;
  bool exitg1;
  double refl_x_obj_local;
  e_struct_T b_asso_list[20];
  int moving_objects;
  double sensor_pos_offset[2];
  double dr_obj;
  double obj_orntn;
  double refl_y_obj_local;
  double refl_x_no_offset;
  double a;
  double b_a;
  double b_obj_list[20];
  int prio2;
  double varargin_1[21];
  if (!isInitialized_radar_tracker) {
    radar_tracker_initialize();
  }

  //  Prediction and transform to sensor CoSy
  for (obj_idx = 0; obj_idx < 50; obj_idx++) {
    if (obj_list[obj_idx].valid) {
    //lnl, brake to stop 
      std::cout<<"----------------A imm_mix------------------"<<std::endl;      
      //  IMM-Models Mixing (a-priori)
      imm_mix(&obj_list[obj_idx]);


    //lnl, brake to stop 
      std::cout<<"----------------B imm_predict------------------"<<std::endl;
      //  Kalman prediction (world coordinates)
      imm_predict(&obj_list[obj_idx], dt);

    //lnl, brake to stop 
      std::cout<<"----------------C locgrid_pre------------------"<<std::endl;
      //  local gridmap predict (forgetting)
      locgrid_pre(&obj_list[obj_idx], dt);

    //lnl, brake to stop 
      std::cout<<"----------------D preproc_egomotion_transform------------------"<<std::endl;
      //  transform to consider ego-motion (tracking is in ego-local coordinate system) 
      preproc_egomotion_transform(&obj_list[obj_idx], meas_list->vx_ego,
        meas_list->psiDt_ego, dt);

    //lnl, brake to stop 
      std::cout<<"----------------E imm_merge------------------"<<std::endl;
      //  merge IMM models for data association
      imm_merge(&obj_list[obj_idx]);

    //lnl, brake to stop 
      std::cout<<"----------------E preproc_flagsinit------------------"<<std::endl;
      //  Initialize flags for multi-sensor fusion (e.g. measureable)
      preproc_flagsinit(&obj_list[obj_idx], meas_list->t,
                        meas_list->dx_sens_offset, meas_list->dy_sens_offset,
                        meas_list->ang_sens_offset, meas_list->fov_range,
                        meas_list->fov_angle, dt);
    }
  }


  //lnl, brake to stop 
  std::cout<<"----------------F Association and measurement update------------------"<<std::endl;
  //  Association and measurement update
  //  Preprocessing: delete multipath and stationary reflections
  preproc_plausi(meas_list);   //lnl,0506/need check
  for (obj_idx2 = 0; obj_idx2 < 1000; obj_idx2++) {
    asso_list[obj_idx2] = r;
  }

  for (obj_idx = 0; obj_idx < 50; obj_idx++) {
    if (obj_list[obj_idx].valid) {
      bool y;
      asso_quick_reject(obj_list[obj_idx].pexist, obj_list[obj_idx].x,
                        obj_list[obj_idx].length, obj_list[obj_idx].width,
                        meas_list->potDoubleRefl, meas_list->meas, meas_list->dr,
                        meas_list->dx_sens_offset, meas_list->dy_sens_offset,
                        meas_reject);
      y = true;
      vxvy_absGate = 0;
      exitg1 = false;
      while ((!exitg1) && (vxvy_absGate < 512)) {
        if (!meas_reject[vxvy_absGate]) {
          y = false;
          exitg1 = true;
        } else {
          vxvy_absGate++;
        }
      }

      if (y) {
        //  execute data association once, in order to fill
        //  obj_list(obj_idx).asso fields
        for (obj_idx2 = 0; obj_idx2 < 20; obj_idx2++) {
          b_asso_list[obj_idx2] = asso_list[obj_idx + 50 * obj_idx2];
        }

        asso_meas2track(&obj_list[obj_idx], meas_list, 1.0, b_asso_list);
        for (obj_idx2 = 0; obj_idx2 < 20; obj_idx2++) {
          asso_list[obj_idx + 50 * obj_idx2] = b_asso_list[obj_idx2];
        }
      } else {
        for (moving_objects = 0; moving_objects < 512; moving_objects++) {
          if (!meas_reject[moving_objects]) {
            //  measurement association
            for (obj_idx2 = 0; obj_idx2 < 20; obj_idx2++) {
              b_asso_list[obj_idx2] = asso_list[obj_idx + 50 * obj_idx2];
            }

            asso_meas2track(&obj_list[obj_idx], meas_list, static_cast<double>
                            (moving_objects) + 1.0, b_asso_list);
            for (obj_idx2 = 0; obj_idx2 < 20; obj_idx2++) {
              asso_list[obj_idx + 50 * obj_idx2] = b_asso_list[obj_idx2];
            }
          }
        }
      }

      if (obj_list[obj_idx].meas) {
        sensor_pos_offset[0] = meas_list->dx_sens_offset;
        sensor_pos_offset[1] = meas_list->dy_sens_offset;

        //  update IMM probabilites
        //  PDA
        //  Kalman measurement update
        for (obj_idx2 = 0; obj_idx2 < 20; obj_idx2++) {
          b_asso_list[obj_idx2] = asso_list[obj_idx + 50 * obj_idx2];
        }

      //lnl, brake to stop 
      std::cout<<"----------------F.1 immpdaf_mup------------------"<<std::endl;
        immpdaf_mup(&obj_list[obj_idx], b_asso_list, sensor_pos_offset,
                    meas_list->vx_ego);

        //  update local gridmap of objects
        //  parameter definitions
        //  load objectproperties
        obj_orntn = obj_list[obj_idx].psi;
        for (int meas_ind = 0; meas_ind < 20; meas_ind++) {
          moving_objects = obj_idx + 50 * meas_ind;
          asso_list[moving_objects] = b_asso_list[meas_ind];
          if (asso_list[moving_objects].loc_nr > 0.0) {
            //  determine indices
            // EKF_6D_CA_MEAS2XY: Conversion of measurement to x-y position
            //  (used for gating etc)
            refl_x_no_offset = (asso_list[moving_objects].y[0] * std::cos
                                (asso_list[moving_objects].y[1]) +
                                sensor_pos_offset[0]) - obj_list[obj_idx].x[0];

            //  both is in global coordinates
            refl_y_no_offset = (asso_list[moving_objects].y[0] * std::sin
                                (asso_list[moving_objects].y[1]) +
                                sensor_pos_offset[1]) - obj_list[obj_idx].x[3];
            refl_y_obj_local = std::sin(obj_orntn);
            dr_obj = std::cos(obj_orntn);
            refl_x_obj_local = dr_obj * refl_x_no_offset + refl_y_obj_local *
              refl_y_no_offset;

            //  in object local cosy
            refl_y_obj_local = -refl_y_obj_local * refl_x_no_offset + dr_obj *
              refl_y_no_offset;
            for (vxvy_absGate = 0; vxvy_absGate < 21; vxvy_absGate++) {
              varargin_1[vxvy_absGate] = std::abs(obj_list[obj_idx]
                .grid.x_cells[vxvy_absGate] - refl_x_obj_local);
            }

            if (!rtIsNaN(varargin_1[0])) {
              prio2 = 1;
            } else {
              prio2 = 0;
              vxvy_absGate = 2;
              exitg1 = false;
              while ((!exitg1) && (vxvy_absGate < 22)) {
                if (!rtIsNaN(varargin_1[vxvy_absGate - 1])) {
                  prio2 = vxvy_absGate;
                  exitg1 = true;
                } else {
                  vxvy_absGate++;
                }
              }
            }

            if (prio2 == 0) {
              prio2 = 1;
            } else {
              refl_x_no_offset = varargin_1[prio2 - 1];
              obj_idx2 = prio2 + 1;
              for (vxvy_absGate = obj_idx2; vxvy_absGate < 22; vxvy_absGate++) {
                refl_y_no_offset = varargin_1[vxvy_absGate - 1];
                if (refl_x_no_offset > refl_y_no_offset) {
                  refl_x_no_offset = refl_y_no_offset;
                  prio2 = vxvy_absGate;
                }
              }
            }

            for (vxvy_absGate = 0; vxvy_absGate < 21; vxvy_absGate++) {
              varargin_1[vxvy_absGate] = std::abs(obj_list[obj_idx]
                .grid.y_cells[vxvy_absGate] - refl_y_obj_local);
            }

            if (!rtIsNaN(varargin_1[0])) {
              moving_objects = 1;
            } else {
              moving_objects = 0;
              vxvy_absGate = 2;
              exitg1 = false;
              while ((!exitg1) && (vxvy_absGate < 22)) {
                if (!rtIsNaN(varargin_1[vxvy_absGate - 1])) {
                  moving_objects = vxvy_absGate;
                  exitg1 = true;
                } else {
                  vxvy_absGate++;
                }
              }
            }

            if (moving_objects == 0) {
              moving_objects = 1;
            } else {
              refl_x_no_offset = varargin_1[moving_objects - 1];
              obj_idx2 = moving_objects + 1;
              for (vxvy_absGate = obj_idx2; vxvy_absGate < 22; vxvy_absGate++) {
                refl_y_no_offset = varargin_1[vxvy_absGate - 1];
                if (refl_x_no_offset > refl_y_no_offset) {
                  refl_x_no_offset = refl_y_no_offset;
                  moving_objects = vxvy_absGate;
                }
              }
            }

            //  update
            obj_idx2 = (prio2 + 21 * (moving_objects - 1)) - 1;
            obj_list[obj_idx].grid.LR[obj_idx2] *= 4.666666666666667;
          }
        }

        //  % plot
        //  likelihood = obj.grid.LR ./ (1 + obj.grid.LR);  % convert odds to prob 
        //  figure(100+obj_idx)
        //  contourf(obj.grid.x_cells,obj.grid.y_cells,likelihood.',100,'linestyle','none') 
        //  caxis([0,1]);
        //  xlabel('x in object cosy')
        //  ylabel('y in object cosy')
        //  xticks(-5:5)
        //  yticks(-5:5)
        //  axis equal
        //  title(sprintf('r = %f',sqrt(obj_px^2+obj_py^2)))
        //  colorbar
        locgrid_recenter(&obj_list[obj_idx]);
      }
    }

    #if tracker_initial
    int count = 0;
    for (int z = 0; z < 20; z++) {
    //  bad matlab programming style, but ok for c-code generation
    if (!(b_asso_list[z].loc_nr == 0.0)) { count++;
    std::cout<<" 88_loc_nr: "<<b_asso_list[z].loc_nr<<std::endl;
    }
    }

    std::cout<<" 88_ID_loc: "<<obj_idx<<" location_number: "<<count<<std::endl;
    #endif
  }

  //  calculate additional attributes
  for (obj_idx = 0; obj_idx < 50; obj_idx++) {
    if (obj_list[obj_idx].valid) {
      //  calculate existence probability
      for (obj_idx2 = 0; obj_idx2 < 20; obj_idx2++) {
        b_asso_list[obj_idx2] = asso_list[obj_idx + 50 * obj_idx2];
      }

      // std::cout<<"pexist before mgmt:"<<obj_idx<<"pexist:"<<obj_list[obj_idx].pexist<<std::endl; //lnl
      //lnl, brake to stop 
      std::cout<<"----------------F.2 mgmt_pex------------------"<<std::endl;
      mgmt_pex(&obj_list[obj_idx], b_asso_list);
      //lnl, brake to stop 
      std::cout<<"----------------F.3 imm_output------------------"<<std::endl;
      //  combine IMM result for output
      imm_output(&obj_list[obj_idx]);

      //  update additional object attributes (e.g. lenght, width, heading, RCS) 
      for (obj_idx2 = 0; obj_idx2 < 20; obj_idx2++) {
        b_asso_list[obj_idx2] = asso_list[obj_idx + 50 * obj_idx2];
      }

      update_derived_values(&obj_list[obj_idx], meas_list->dBRcs, b_asso_list);

      //  object classification
      //  [ obj_list(obj_idx) ] = lkf_6D_classi( obj_list(obj_idx) );
    }
  }

  //  delete invalid objects
  refl_y_no_offset = meas_list->vx_ego;

  //  delete irrelevant objects
  //  merge nearby objects
  refl_x_obj_local = 0.0;
  for (obj_idx = 0; obj_idx < 50; obj_idx++) {
    if (obj_list[obj_idx].valid) {
      refl_x_obj_local++;
    }
  }

  for (obj_idx = 0; obj_idx < 50; obj_idx++) {
    if (obj_list[obj_idx].valid) {
      double vr_obj;
      dr_obj = std::sqrt(obj_list[obj_idx].x[0] * obj_list[obj_idx].x[0] +
                         obj_list[obj_idx].x[3] * obj_list[obj_idx].x[3]);
      obj_orntn = rt_atan2d_snf(obj_list[obj_idx].x[3], obj_list[obj_idx].x[0]);
      vr_obj = std::cos(obj_orntn) * (obj_list[obj_idx].x[1] - refl_y_no_offset)
        + std::sin(obj_orntn) * obj_list[obj_idx].x[4];
      refl_y_obj_local = dr_obj / 10.0;
      if ((5.0 > refl_y_obj_local) || rtIsNaN(refl_y_obj_local)) {
        refl_y_obj_local = 5.0;
      }

      //  max of 5m and range/10
      //  delete unlikely objects
      // if (obj_list[obj_idx].pexist < 0.025) {
      //   get_clean_obj(&obj_list[obj_idx]);
      // } else if ((obj_list[obj_idx].pexist < 0.15) && (refl_x_obj_local > 30.0))
      // {
      //   get_clean_obj(&obj_list[obj_idx]);
      // } else if ((obj_list[obj_idx].pexist < 0.2) && (refl_x_obj_local > 45.0))
      // {
      //   get_clean_obj(&obj_list[obj_idx]); //lnl
      // std::cout<<"this is the pexist for"<<obj_idx<<":"<<obj_list[obj_idx].pexist<<"and refl:"<<refl_x_obj_local<<std::endl;
      // bool stay_observe = false;
      bool stay_observe = false;
      if (obj_list[obj_idx].age>0.06)//lnl, age adjustment 0.066
      {
        stay_observe = true;
      }
      // if(stay_observe)
      // {//lnl, six get clean
      if (obj_list[obj_idx].pexist < 0.5&&stay_observe)  {  //0.025 is adjustable
        get_clean_obj(&obj_list[obj_idx]);
      } else if ((obj_list[obj_idx].pexist < 0.15) && (refl_x_obj_local > 30.0)&&stay_observe)
      {
        get_clean_obj(&obj_list[obj_idx]);
      } else if ((obj_list[obj_idx].pexist < 0.2) && (refl_x_obj_local > 45.0)&&stay_observe)
      {
        get_clean_obj(&obj_list[obj_idx]);

        //  delete objects which are not updated (out of FOV)
      // } else if (std::abs(obj_list[obj_idx].t_lastmeas - meas_list->t) > 0.75) {
      //   //  s
        // get_clean_obj(&obj_list[obj_idx]);
      //lnl, test, radar performance
      } else if (std::abs(obj_list[obj_idx].t_lastmeas - meas_list->t) > 0.75&&stay_observe) { //0.75
        //  s
        get_clean_obj(&obj_list[obj_idx]); //lnl

        //  delete objects outside region of interest
      } else if (dr_obj > 250.0) {    //lnl, distance filter !!
        //  be careful: depends on sensor type! //lnl, make sense for unreal crossing objects
        get_clean_obj(&obj_list[obj_idx]);
      // } else if (dr_obj < 40.0) {    //lnl, distance filter !! added 
      //   //  be careful: depends on sensor type! //lnl, make sense for unreal crossing objects
      //   get_clean_obj(&obj_list[obj_idx]);
      } else if (((obj_list[obj_idx].P[0] > refl_y_obj_local * refl_y_obj_local) ||
                 (obj_list[obj_idx].P[21] > refl_y_obj_local * refl_y_obj_local))&&(stay_observe))
      {
        //  object uncertainty for position //lnl, need adjust for threshold refl_y_obj_local^2
        get_clean_obj(&obj_list[obj_idx]);
      } else {
        //  Merge nearby objects
        obj_idx2 = 0;
        exitg1 = false;
        while ((!exitg1) && (obj_idx2 < 50)) {
          if (obj_list[obj_idx2].valid && (obj_idx2 != obj_idx)) {
            //  Delete / Merge ---------------------------------------------------------------- 
            if ((obj_list[obj_idx].moving >= 0.9) && (obj_list[obj_idx2].moving >=
                 0.9)) {
              moving_objects = 1;
              prio2 = 5;
              vxvy_absGate = 3;
            } else {
              moving_objects = 0;
              prio2 = 1;
              vxvy_absGate = 1;
            }

            refl_y_obj_local = rt_atan2d_snf(obj_list[obj_idx2].x[3],
              obj_list[obj_idx2].x[0]);
            refl_x_no_offset = std::abs(obj_orntn - refl_y_obj_local);
            if (refl_x_no_offset > 3.1415926535897931) {
              refl_x_no_offset = 6.2831853071795862 - refl_x_no_offset;
            }

            //  are objects nearby?
            //      % are objects separable by radar sensor?
            a = obj_list[obj_idx].x[0] - obj_list[obj_idx2].x[0];
            b_a = obj_list[obj_idx].x[3] - obj_list[obj_idx2].x[3];

            //###################lnl### Cal projection ###//
            auto sym_vxvy_idx = obj_list[obj_idx].x[1]*obj_list[obj_idx].x[4];
            auto sym_vxvy_idx2 = obj_list[obj_idx2].x[1]*obj_list[obj_idx2].x[4];
            auto sym_pos = a*b_a;
            double proj_ang_to_idx;
            double proj_ang_to_idx2;
            //to idx
            if(sym_vxvy_idx*sym_pos<0)
            {
              proj_ang_to_idx = std::atan(std::abs(std::tan(obj_list[obj_idx].psi))) + rt_atan2d_snf(std::abs(b_a),std::abs(a));
            }
            else 
            {
              proj_ang_to_idx = std::abs(std::atan(std::abs(std::tan(obj_list[obj_idx].psi))) - rt_atan2d_snf(std::abs(b_a),std::abs(a)));  //lnl, added abs()
            }

            auto proj_x_dis_to_idx = std::sqrt(a*a + b_a*b_a)*std::cos(proj_ang_to_idx);
            auto proj_y_dis_to_idx = std::sqrt(a*a + b_a*b_a)*std::sin(proj_ang_to_idx);

            auto delta_idx_idx2 = obj_list[obj_idx].psi - obj_list[obj_idx2].psi;    //not correct?, lnl !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // std::cout<<" 000_delta_idx_idx2: "<<delta_idx_idx2<<std::endl;
            auto proj_x_idx2_to_idx = 0.5*(obj_list[obj_idx2].length*std::abs(std::cos(delta_idx_idx2)) + obj_list[obj_idx2].width*std::abs(std::sin(delta_idx_idx2)));
            auto proj_y_idx2_to_idx = 0.5*(obj_list[obj_idx2].length*std::abs(std::sin(delta_idx_idx2)) + obj_list[obj_idx2].width*std::abs(std::cos(delta_idx_idx2)));

            // bool x_idx = (proj_x_idx2_to_idx + obj_list[obj_idx].length)<proj_x_dis_to_idx;     //lnl, wrong, used to be width
            // bool y_idx = (proj_y_idx2_to_idx + obj_list[obj_idx].width)<proj_y_dis_to_idx;
            //to idx2
            if(sym_vxvy_idx2*sym_pos<0)
            {
              proj_ang_to_idx2 = std::atan(std::abs(std::tan(obj_list[obj_idx2].psi))) + rt_atan2d_snf(std::abs(b_a),std::abs(a));
            }
            else 
            {
              proj_ang_to_idx2 = std::atan(std::abs(std::tan(obj_list[obj_idx2].psi))) - rt_atan2d_snf(std::abs(b_a),std::abs(a));
            }

            auto proj_x_dis_to_idx2 = std::sqrt(a*a + b_a*b_a)*std::cos(proj_ang_to_idx2);
            auto proj_y_dis_to_idx2 = std::sqrt(a*a + b_a*b_a)*std::sin(proj_ang_to_idx2);

            // auto delta_idx_idx2 = obj_list[obj_idx].psi - obj_list[obj_idx2].psi;
            auto proj_x_idx_to_idx2 = 0.5*(obj_list[obj_idx].length*std::abs(std::cos(-delta_idx_idx2)) + obj_list[obj_idx].width*std::abs(std::sin(-delta_idx_idx2)));
            auto proj_y_idx_to_idx2 = 0.5*(obj_list[obj_idx].length*std::abs(std::sin(-delta_idx_idx2)) + obj_list[obj_idx].width*std::abs(std::cos(-delta_idx_idx2)));

            // bool x_idx2 = (proj_x_idx_to_idx2 + obj_list[obj_idx2].length)<proj_x_dis_to_idx2;
            // bool y_idx2 = (proj_y_idx_to_idx2 + obj_list[obj_idx2].width)<proj_y_dis_to_idx2;
            double delta_x_idx{-100}, delta_y_idx{-100}, delta_x_idx2{-100}, delta_y_idx2{-100};

            // if(x_idx)
            // {
            //   delta_x_idx = proj_x_dis_to_idx - (proj_x_idx2_to_idx + obj_list[obj_idx].width);
            //   std::cout<<"delta_x_idx: "<<delta_x_idx<<std::endl;
            // }
            // if(y_idx)
            // {
            //   delta_y_idx = proj_y_dis_to_idx - (proj_y_idx2_to_idx + obj_list[obj_idx].length);
            //   std::cout<<"delta_y_idx: "<<delta_y_idx<<std::endl;
            // }            
            // if(x_idx2)
            // {
            //   delta_x_idx2 = proj_x_dis_to_idx2 - (proj_y_idx_to_idx2 + obj_list[obj_idx2].width);
            //   std::cout<<"delta_x_idx2: "<<delta_x_idx2<<std::endl;
            // }
            // if(y_idx2)
            // {
            //   delta_y_idx2 = proj_y_dis_to_idx2 - (proj_y_idx_to_idx2 + obj_list[obj_idx2].length);
            //   std::cout<<"delta_y_idx2: "<<delta_y_idx2<<std::endl;
            // }
            // std::cout<<" x_idx: "<<x_idx<<" y_idx: "<<y_idx<<" x_idx2: "<<x_idx2<<" y_idx2: "<<y_idx2<<std::endl;


            delta_x_idx = proj_x_dis_to_idx - (proj_x_idx2_to_idx + obj_list[obj_idx].length);  //lnl, used to be width
            delta_y_idx = proj_y_dis_to_idx - (proj_y_idx2_to_idx + obj_list[obj_idx].width);
            delta_x_idx2 = proj_x_dis_to_idx2 - (proj_x_idx_to_idx2 + obj_list[obj_idx2].length);
            delta_y_idx2 = proj_y_dis_to_idx2 - (proj_y_idx_to_idx2 + obj_list[obj_idx2].width);

            // std::cout<<"delta_x_idx: "<<delta_x_idx<<std::endl;
            // std::cout<<"delta_y_idx: "<<delta_y_idx<<std::endl;
            // std::cout<<"delta_x_idx2: "<<delta_x_idx2<<std::endl;
            // std::cout<<"delta_y_idx2: "<<delta_y_idx2<<std::endl;

            //calc correction to objects incase, overlap exist for bounding boxes.
            //delta list [delta_x_idx,delta_y_idx,delta_x_idx2,delta_y_idx2]
            std::vector<double> delta_list = {delta_x_idx,delta_y_idx,delta_x_idx2,delta_y_idx2};
            double overlap_threshold1{0.1};
            double overlap_threshold2{-0.6};
            double correc_coef_idx{0.3};
            double correc_coef_idx2{0.8};
            bool enable_correction {true};

            // double x_correctiondelta_correction{0};
            int delta_index{-1};
            double x_correction{0}, y_correction{0};

            double delta_correction = delta_list[0];


            delta_index = 0;

            // std::cout<<" sizeof(delta_list): "<<delta_list.size()<<std::endl;
            //calc max projection distance
            for(int i=0;i<delta_list.size()-1;i++)
            {             
              if (delta_correction<delta_list[i+1])
              {
                delta_correction = delta_list[i+1];
                delta_index = i+1;
                // std::cout<<" (delta_index): "<<delta_index<<std::endl;
              }            
            }
            // std::cout<<" delta_correction: "<<delta_correction<<" delta_index: "<<delta_index<<std::endl;

            obj_list[obj_idx].test_delta = delta_correction;
            obj_list[obj_idx2].test_delta = delta_correction;
            if(enable_correction)
            {
            if((delta_correction<overlap_threshold1)&&(delta_correction>=overlap_threshold2))
            {          
              switch(delta_index)
              {
                case 0:
                  x_correction = std::abs(delta_correction/std::cos(proj_ang_to_idx)*std::cos(rt_atan2d_snf(std::abs(b_a),std::abs(a))));
                  y_correction = std::abs(delta_correction/std::cos(proj_ang_to_idx)*std::sin(rt_atan2d_snf(std::abs(b_a),std::abs(a))));
                  break;
                case 1:
                  x_correction = std::abs(delta_correction/std::sin(proj_ang_to_idx)*std::cos(rt_atan2d_snf(std::abs(b_a),std::abs(a))));
                  y_correction = std::abs(delta_correction/std::sin(proj_ang_to_idx)*std::sin(rt_atan2d_snf(std::abs(b_a),std::abs(a))));
                  break;
                case 2:
                  x_correction = std::abs(delta_correction/std::cos(proj_ang_to_idx2)*std::cos(rt_atan2d_snf(std::abs(b_a),std::abs(a))));
                  y_correction = std::abs(delta_correction/std::cos(proj_ang_to_idx2)*std::sin(rt_atan2d_snf(std::abs(b_a),std::abs(a)))); 
                  break;
                case 3:
                  x_correction = std::abs(delta_correction/std::sin(proj_ang_to_idx2)*std::cos(rt_atan2d_snf(std::abs(b_a),std::abs(a))));
                  y_correction = std::abs(delta_correction/std::sin(proj_ang_to_idx2)*std::sin(rt_atan2d_snf(std::abs(b_a),std::abs(a)))); 
                  break; 
                default:
                  break;                                        
              }
              // std::cout<<" x_correction: "<<x_correction<<" y_correction: "<<y_correction<<std::endl;

              if(a>0)
              {
                obj_list[obj_idx].x[0] = obj_list[obj_idx].x[0] + correc_coef_idx*x_correction;
                obj_list[obj_idx2].x[0] = obj_list[obj_idx2].x[0] - correc_coef_idx2*x_correction;
              }
              else
              {
                obj_list[obj_idx].x[0] = obj_list[obj_idx].x[0] - correc_coef_idx*x_correction;
                obj_list[obj_idx2].x[0] = obj_list[obj_idx2].x[0] + correc_coef_idx2*x_correction;
              }
              
              if(b_a>0)
              {
                obj_list[obj_idx].x[3] = obj_list[obj_idx].x[3] + correc_coef_idx*y_correction;
                obj_list[obj_idx2].x[3] = obj_list[obj_idx2].x[3] - correc_coef_idx2*y_correction;              
              }
              else
              {
                obj_list[obj_idx].x[3] = obj_list[obj_idx].x[3] - correc_coef_idx*y_correction;
                obj_list[obj_idx2].x[3] = obj_list[obj_idx2].x[3] + correc_coef_idx2*y_correction;   
              }
            }
            }


            // //auto delta_correction = std::max({delta_x_idx,delta_y_idx,delta_x_idx2,delta_y_idx2});

            // //auto x_correction = std::abs(delta_correction/std::cos(proj_ang_to_idx)*std::cos(rt_atan2d_snf(std::abs(b_a),std::abs(a))));
            // //auto y_correction = std::abs(delta_correction/std::cos(proj_ang_to_idx)*std::sin(rt_atan2d_snf(std::abs(b_a),std::abs(a))));


            //######################### Cal projection ###//

            // std::sqrt(a*a + b_a*b_a)
            //lnl, test
            // if (((a * a + b_a * b_a < prio2 * prio2) && (std::abs
            //       (obj_list[obj_idx].x[1] - obj_list[obj_idx2].x[1]) <
            //       vxvy_absGate) && (std::abs(obj_list[obj_idx].x[4] -
            //        obj_list[obj_idx2].x[4]) < vxvy_absGate)) || ((std::abs
            //       (dr_obj - std::sqrt(obj_list[obj_idx2].x[0] *
            //         obj_list[obj_idx2].x[0] + obj_list[obj_idx2].x[3] *
            //         obj_list[obj_idx2].x[3])) < 0.2) && (std::abs(vr_obj - (std::
            //         cos(refl_y_obj_local) * (obj_list[obj_idx2].x[1] -
            //          refl_y_no_offset) + std::sin(refl_y_obj_local) *
            //         obj_list[obj_idx2].x[4])) < 0.1) && (refl_x_no_offset <
            //       0.17453292519943295) && (moving_objects == 1))){
            // if (
            //     ((a * a + b_a * b_a < 0.5*0.5) && (std::abs
            //       (obj_list[obj_idx].x[1] - obj_list[obj_idx2].x[1]) <
            //       vxvy_absGate) && (std::abs(obj_list[obj_idx].x[4] -
            //        obj_list[obj_idx2].x[4]) < vxvy_absGate))
            //        ||
            //       (
            //         (std::abs(dr_obj - std::sqrt(obj_list[obj_idx2].x[0] *
            //         obj_list[obj_idx2].x[0] + obj_list[obj_idx2].x[3] *
            //         obj_list[obj_idx2].x[3])) < 0.2)&&                    
            //         (std::abs(vr_obj - (std::
            //         cos(refl_y_obj_local) * (obj_list[obj_idx2].x[1] -
            //          refl_y_no_offset) + std::sin(refl_y_obj_local) *
            //         obj_list[obj_idx2].x[4])) < 0.1) && (refl_x_no_offset <
            //       0.17453292519943295) && (moving_objects == 1)                   
            //        )
            //        ){
            //delta_correction<overlap_threshold2

            if (
                ((delta_correction<-0.6) 
                && (std::abs
                  (obj_list[obj_idx].x[1] - obj_list[obj_idx2].x[1]) <
                  vxvy_absGate) && (std::abs(obj_list[obj_idx].x[4] -
                   obj_list[obj_idx2].x[4]) < vxvy_absGate)
                   )
                  //  ||
                  // (
                  //   (std::abs(dr_obj - std::sqrt(obj_list[obj_idx2].x[0] *
                  //   obj_list[obj_idx2].x[0] + obj_list[obj_idx2].x[3] *
                  //   obj_list[obj_idx2].x[3])) < 0.2)&&                    
                  //   (std::abs(vr_obj - (std::
                  //   cos(refl_y_obj_local) * (obj_list[obj_idx2].x[1] -
                  //    refl_y_no_offset) + std::sin(refl_y_obj_local) *
                  //   obj_list[obj_idx2].x[4])) < 0.1) && (refl_x_no_offset <
                  // 0.17453292519943295) && (moving_objects == 1)                  
                  //  )
                   ){
            // //lnl, test
            // if (0) {
              moving_objects = 0;
              prio2 = 0;
              if (obj_list[obj_idx].pexist > obj_list[obj_idx2].pexist + 0.05) {
                //  higher p_exist has priority
                moving_objects = 1;
              }

              if (obj_list[obj_idx2].pexist > obj_list[obj_idx].pexist + 0.05) {
                //  higher p_exist has priority
                prio2 = 1;
              }

              if (obj_list[obj_idx].moving > obj_list[obj_idx2].moving) {
                //  moving objects have priority
                moving_objects++;
              }

              if (obj_list[obj_idx2].moving > obj_list[obj_idx].moving) {
                //  moving objects have priority
                prio2++;
              }
              auto const age_du = obj_list[obj_idx].age - obj_list[obj_idx2].age;  //age, lnl
              // std::cout<<"000_age_duration:"<<age_du<<std::endl;
              if (obj_list[obj_idx].age > obj_list[obj_idx2].age) {
                //  old objects have priority
                /* lnl */
                moving_objects++;
                // moving_objects = moving_objects +10*(age_du/(age_du+1));
                moving_objects = moving_objects +3*(age_du/(age_du+1));
              }

              if (obj_list[obj_idx2].age > obj_list[obj_idx].age) {
                //  old objects have priority
                 /* lnl */
                prio2++;
                // prio2 = prio2 + 10*(age_du/(age_du+1));
                prio2 = prio2 + 3*(age_du/(age_du+1));
              }

              //  delete objects with lower prio. If prio is equal, the age decides 
              if (moving_objects > prio2) {
                get_clean_obj(&obj_list[obj_idx2]);
                obj_idx2++;
              } else if (prio2 > moving_objects) {
                get_clean_obj(&obj_list[obj_idx]);
                exitg1 = true;
              } else {
                obj_idx2++;
              }
            } else {
              obj_idx2++;
            }
          } else {
            obj_idx2++;
          }
        }
      }
      // }//lnl
    }
  }

  //  create new tracks from unassociated measurements   //lnl, 0506, need to add
  mgmt_init_track(obj_list, meas_list->asso, meas_list->potDoubleRefl,
                  meas_list->meas, meas_list->dr, meas_list->vr,
                  meas_list->dBRcs, meas_list->t, meas_list->vx_ego,
                  meas_list->dx_sens_offset, meas_list->dy_sens_offset,
                  meas_list->alpha, dt);

  //  update trajectory history
  for (obj_idx = 0; obj_idx < 50; obj_idx++) {
    //  % && (obj_list(obj_idx).meas == true) ...
    if (obj_list[obj_idx].valid && ((obj_list[obj_idx].moving > 0.95) ||
         (obj_list[obj_idx].traj_hist_len > 0.0))) {

        // std::cout<<" obj_list[obj_idx].test_delta  delta_correction: "<<obj_list[obj_idx].test_delta<<std::endl;
      //  always update if once updated
      //  Start with smaller distances, to have quickly a full buffer $/
      if (obj_list[obj_idx].traj_hist_len < 5.0) {
        dr_obj = 1.0;
      } else if (obj_list[obj_idx].traj_hist_len < 10.0) {
        dr_obj = 2.0;
      } else {
        refl_y_obj_local = 0.2 * std::sqrt(obj_list[obj_idx].x[1] *
          obj_list[obj_idx].x[1] + obj_list[obj_idx].x[4] * obj_list[obj_idx].x
          [4]);
        if ((2.0 > refl_y_obj_local) || rtIsNaN(refl_y_obj_local)) {
          dr_obj = 2.0;
        } else {
          dr_obj = refl_y_obj_local;
        }
      }

      //  center of rear axis
      refl_x_no_offset = -0.5 * obj_list[obj_idx].length;
      refl_y_no_offset = refl_x_no_offset * std::cos(obj_list[obj_idx].
        psi_traj_hist) * 0.0;
      refl_y_obj_local = refl_x_no_offset * std::sin(obj_list[obj_idx].
        psi_traj_hist) * 0.0;
      if (obj_list[obj_idx].traj_hist_len == 0.0) {
        obj_list[obj_idx].traj_hist_x[0] = obj_list[obj_idx].x[0] +
          refl_y_no_offset;

        //  + obj_list(obj_idx).x_refl(1);
        obj_list[obj_idx].traj_hist_y[0] = obj_list[obj_idx].x[3] +
          refl_y_obj_local;

        //  + obj_list(obj_idx).y_refl(1);
        obj_list[obj_idx].traj_hist_vx[0] = obj_list[obj_idx].x[1];
        obj_list[obj_idx].traj_hist_vy[0] = obj_list[obj_idx].x[4];
        obj_list[obj_idx].traj_hist_t[0] = obj_list[obj_idx].t_abs;
        obj_list[obj_idx].traj_hist_len = 1.0;
      } else {
        refl_x_no_offset = obj_list[obj_idx].x[0] + refl_y_no_offset;
        a = refl_x_no_offset - obj_list[obj_idx].traj_hist_x[0];
        b_a = (obj_list[obj_idx].x[3] + refl_y_obj_local) - obj_list[obj_idx].
          traj_hist_y[0];
        if (a * a + b_a * b_a > dr_obj * dr_obj) {
          //  minimum distance traveled or new object
          b_obj_list[0] = refl_x_no_offset;
          std::memcpy(&b_obj_list[1], &obj_list[obj_idx].traj_hist_x[0], 19U *
                      sizeof(double));
          std::memcpy(&obj_list[obj_idx].traj_hist_x[0], &b_obj_list[0], 20U *
                      sizeof(double));

          //  + obj_list(obj_idx).x_refl(1);
          b_obj_list[0] = obj_list[obj_idx].x[3] + refl_y_obj_local;
          std::memcpy(&b_obj_list[1], &obj_list[obj_idx].traj_hist_y[0], 19U *
                      sizeof(double));
          std::memcpy(&obj_list[obj_idx].traj_hist_y[0], &b_obj_list[0], 20U *
                      sizeof(double));

          //  + obj_list(obj_idx).y_refl(1);
          b_obj_list[0] = obj_list[obj_idx].x[1];
          std::memcpy(&b_obj_list[1], &obj_list[obj_idx].traj_hist_vx[0], 19U *
                      sizeof(double));
          std::memcpy(&obj_list[obj_idx].traj_hist_vx[0], &b_obj_list[0], 20U *
                      sizeof(double));
          b_obj_list[0] = obj_list[obj_idx].x[4];
          std::memcpy(&b_obj_list[1], &obj_list[obj_idx].traj_hist_vy[0], 19U *
                      sizeof(double));
          std::memcpy(&obj_list[obj_idx].traj_hist_vy[0], &b_obj_list[0], 20U *
                      sizeof(double));
          b_obj_list[0] = obj_list[obj_idx].t_abs;
          std::memcpy(&b_obj_list[1], &obj_list[obj_idx].traj_hist_vx[0], 19U *
                      sizeof(double));
          std::memcpy(&obj_list[obj_idx].traj_hist_vx[0], &b_obj_list[0], 20U *
                      sizeof(double));
          if ((20.0 < obj_list[obj_idx].traj_hist_len + 1.0) || rtIsNaN
              (obj_list[obj_idx].traj_hist_len + 1.0)) {
            obj_list[obj_idx].traj_hist_len = 20.0;
          } else {
            obj_list[obj_idx].traj_hist_len++;
          }
        }
      }
    }
  }
}

//
// File trailer for radar_tracker.cpp
//
// [EOF]
//
