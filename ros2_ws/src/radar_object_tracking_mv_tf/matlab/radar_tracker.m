function  obj_list = radar_tracker(obj_list, meas_list, dt) %#codegen
% tracking loop with 6D Kalman filter
%
% x: state vector [dx vx ax dy vy ay]';
% P: covariance matrix of x
% y: measurement vector [dr vr alpha]';
% R: covariance matrix of y
%
coder.cstructname(obj_list, 'OBJECT_STRUCT');
coder.cstructname(meas_list, 'MEASLIST_STRUCT');
coder.inline('never');

coder.extrinsic('fprintf');


%% Prediction and transform to sensor CoSy
for obj_idx = 1:length(obj_list) 
    if obj_list(obj_idx).valid
        
        % IMM-Models Mixing (a-priori)
        obj_list(obj_idx) = imm_mix(obj_list(obj_idx), meas_list.vx_ego);

        % Kalman prediction (world coordinates)
        obj_list(obj_idx) = imm_predict(obj_list(obj_idx),dt);
        
        % local gridmap predict (forgetting)
        obj_list(obj_idx) = locgrid_pre(obj_list(obj_idx),dt);

        % transform to consider ego-motion (tracking is in ego-local coordinate system)
        obj_list(obj_idx) = preproc_egomotion_transform(obj_list(obj_idx),meas_list,dt);
        
        % merge IMM models for data association
        obj_list(obj_idx) = imm_merge(obj_list(obj_idx), meas_list.vx_ego);             
        
        % Initialize flags for multi-sensor fusion (e.g. measureable)
        obj_list(obj_idx) = preproc_flagsinit(obj_list(obj_idx), meas_list, dt);
    end
end


%% Association and measurement update

% Preprocessing: delete multipath and stationary reflections 
meas_list = preproc_plausi(meas_list);

asso_list = get_clean_asso_list();
for obj_idx = 1:length(obj_list) 
    if obj_list(obj_idx).valid 
        meas_reject = asso_quick_reject(obj_list(obj_idx), meas_list);
        if all(meas_reject)
            % execute data association once, in order to fill
            % obj_list(obj_idx).asso fields
            [obj_list(obj_idx), meas_list, asso_list(obj_idx,:)] = asso_meas2track(obj_list(obj_idx), meas_list, 1, asso_list(obj_idx,:));                 
        else
            for idx_meas = 1:length(meas_list.dr)
                if meas_reject(idx_meas)
                    continue
                end
                % measurement association
                [obj_list(obj_idx), meas_list, asso_list(obj_idx,:)] = asso_meas2track(obj_list(obj_idx), meas_list, idx_meas, asso_list(obj_idx,:));                    
            end
        end

        if obj_list(obj_idx).meas
            sensor_pos_offset = [meas_list.dx_sens_offset; meas_list.dy_sens_offset];
            
            % update IMM probabilites
            % PDA
            % Kalman measurement update
            [obj_list(obj_idx), asso_list(obj_idx,:)] = immpdaf_mup(obj_list(obj_idx),asso_list(obj_idx,:), sensor_pos_offset, meas_list.vx_ego);
            
            % update local gridmap of objects
            obj_list(obj_idx) = locgrid_mup(obj_list(obj_idx), asso_list(obj_idx,:), obj_idx, sensor_pos_offset);
            obj_list(obj_idx) = locgrid_recenter(obj_list(obj_idx));
        end
    end
end


%% calculate additional attributes
for obj_idx = 1:length(obj_list)
    if obj_list(obj_idx).valid 
        % calculate existence probability
        obj_list(obj_idx) = mgmt_pex(obj_list(obj_idx), asso_list(obj_idx,:));

        % combine IMM result for output
        obj_list(obj_idx) = imm_output(obj_list(obj_idx),meas_list.vx_ego);  
        assert(all(isfinite(obj_list(obj_idx).x)))
        
        % update additional object attributes (e.g. lenght, width, heading, RCS)
        obj_list(obj_idx) = update_derived_values(obj_list(obj_idx),meas_list,asso_list(obj_idx,:));
        
        % object classification
        % [ obj_list(obj_idx) ] = lkf_6D_classi( obj_list(obj_idx) );
    end
end


%% delete invalid objects
[obj_list] = mgmt_del(obj_list, meas_list.vx_ego, meas_list.t);

%% create new tracks from unassociated measurements
[obj_list] = mgmt_init_track(obj_list, meas_list, dt);


%% update trajectory history
[obj_list] = trajhist_update(obj_list);


for obj_idx = 1:length(obj_list)
    if obj_list(obj_idx).valid 
        assert(all(isfinite(obj_list(obj_idx).x)))
    end
end
