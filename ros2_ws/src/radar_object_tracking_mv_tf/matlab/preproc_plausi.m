function [meas_list] = preproc_plausi(meas_list)    %#codegen
% Preprocessing of the measurement data 

if abs(meas_list.psiDt_ego) < 0.05
    % estimate vx_ego from locations
    meas_v = meas_list.vr(meas_list.meas == 1);
    meas_ang = meas_list.alpha(meas_list.meas == 1);
    vx_ego = estimate_vxego_ransac(meas_v, meas_ang, 100, 10, 0.1);  
    delta_vx = abs(vx_ego - meas_list.vx_ego);
    % Avoid, that overwrite with a completely wrong velocity
    if delta_vx < max(0.8, 0.1 * abs(meas_list.vx_ego))
        meas_list.vx_ego = vx_ego;        
    else
        % disp('that did not work')
    end
end

v_t_yaw_rate = - meas_list.psiDt_ego * sqrt(meas_list.dx_sens_offset^2 + meas_list.dy_sens_offset^2);
sens_mount_angle = atan2(meas_list.dy_sens_offset, meas_list.dx_sens_offset);
vx_ego = meas_list.vx_ego + v_t_yaw_rate * sin(sens_mount_angle); 
vy_ego = - v_t_yaw_rate * cos(sens_mount_angle);

% plausibilisation of locations
for idx1_meas = 1:(length(meas_list.dr)-1)
    if (meas_list.meas(idx1_meas) == 1)
        % Delete multipath reflections
        if meas_list.dr(idx1_meas) < 30
            for idx2_meas = (idx1_meas+1):length(meas_list.dr)
                if (meas_list.meas(idx2_meas) == 1)
                    if meas_list.dr(idx2_meas) < 30

                        if (abs(meas_list.dr(idx1_meas) - 0.5*meas_list.dr(idx2_meas)) < 1.0) 
                            if (abs(meas_list.vr(idx1_meas) - 0.5*meas_list.vr(idx2_meas)) < 0.5)
                                valid_angles1 = isfinite(meas_list.alpha(:,idx1_meas));
                                valid_angles2 = isfinite(meas_list.alpha(:,idx2_meas));
                                ang1 = mean(meas_list.alpha(valid_angles1,idx1_meas));
                                ang2 = mean(meas_list.alpha(valid_angles2,idx2_meas));
                                if (abs(ang1 - ang2) < (10/180*pi))
                                    meas_list.potDoubleRefl(idx2_meas) = 1;
                                    meas_list.meas(idx2_meas) = 0;
                                end
                            end
                        elseif (abs(meas_list.dr(idx2_meas) - 0.5*meas_list.dr(idx1_meas)) < 1.0) 
                            if (abs(meas_list.vr(idx2_meas) - 0.5*meas_list.vr(idx1_meas)) < 0.5)
                                valid_angles1 = isfinite(meas_list.alpha(:,idx1_meas));
                                valid_angles2 = isfinite(meas_list.alpha(:,idx2_meas));
                                ang1 = mean(meas_list.alpha(valid_angles1,idx1_meas));
                                ang2 = mean(meas_list.alpha(valid_angles2,idx2_meas));
                                if (abs(ang1 - ang2) < (10/180*pi))
                                    meas_list.potDoubleRefl(idx1_meas) = 1;
                                    meas_list.meas(idx1_meas) = 0;
                                    break;
                                end
                            end
                        end
                        
                    end
                end
            end 
        end
        
        % discard stationary locations
        vr_ego_unamb = - (vx_ego * cos(meas_list.alpha(idx1_meas)) + vy_ego * sin(meas_list.alpha(idx1_meas)));
      
        delta_v_standing_abs_unamb = abs(meas_list.vr(idx1_meas) - vr_ego_unamb);

        if  delta_v_standing_abs_unamb < max(0.8, 0.1 * abs(meas_list.vx_ego)) %0.8
            meas_list.meas(idx1_meas) = 0;  % ignore stationary locations
        end
        
        if meas_list.vx_ego > 0.1
            
            px_global = meas_list.dr(idx1_meas)*cos(meas_list.alpha(idx1_meas)) + meas_list.dx_sens_offset;
            py_global = meas_list.dr(idx1_meas)*sin(meas_list.alpha(idx1_meas)) + meas_list.dy_sens_offset;
            
            angle_deviation_from_x_axis_global = atan(py_global/px_global); % intentionally NOT use atan2; if loc behind us, we want to get 
            
            location_in_front = (abs(py_global) < 2) && (px_global > 0); % location in front of us (on lane?)
            location_behind = (abs(py_global) < 2) && (px_global < 0); % location behind us (on lane?)
            if meas_list.dr(idx1_meas) < 20 % near range
                if location_in_front
                    if (meas_list.vr(idx1_meas) < 0) && (meas_list.vr(idx1_meas) > (- meas_list.vx_ego *1.1)) && (meas_list.vr(idx1_meas) < (- meas_list.vx_ego *0.3))
                        % probably a ground reflex
                        meas_list.meas(idx1_meas) = 0;
                    end
                elseif location_behind
                    if (meas_list.vr(idx1_meas) > 0) && (meas_list.vr(idx1_meas) < (meas_list.vx_ego *1.1)) && (meas_list.vr(idx1_meas) > (meas_list.vx_ego *0.3))
                        % probably a ground reflex
                        meas_list.meas(idx1_meas) = 0;
                    end
                end
            end
            
            if (not(location_in_front) && not(location_behind)) || meas_list.dr(idx1_meas)>20 % location not on our lane
                if abs(angle_deviation_from_x_axis_global) > (10/180*pi) % exclude follower vehicle
                    % Gen5 corner B1.2 sample has 45-degree ambiguities, due to antenna design.
                    % We try to find the angle, which would fit best for an ego-motion compensation. 
                    % (Assuming target is stationary)
                    possible_angle_offsets = [-pi/2; 0; pi/2];  % 90-degree ambiguity

                    vr_ego = - (vx_ego * cos(meas_list.alpha(idx1_meas) + possible_angle_offsets) + ...
                        vy_ego * sin(meas_list.alpha(idx1_meas) + possible_angle_offsets));
                    delta_v_standing_abs = abs(meas_list.vr(idx1_meas) - vr_ego);

                    [smallest_v_standing_abs, best_angle_offset_ind] = min(delta_v_standing_abs);

                    if  smallest_v_standing_abs < 0.6
                        meas_list.meas(idx1_meas) = 0;  % ignore stationary locations

                        meas_list.alpha(idx1_meas) = meas_list.alpha(idx1_meas) + possible_angle_offsets(best_angle_offset_ind);
                    end % if ego-motion compensation not good enough (i.e. moving target), do nothing
                end
            end           
        end
        
        if abs(meas_list.vr(idx1_meas)) < 0.02
            % probably a ground reflex
            meas_list.meas(idx1_meas) = 0;
        end
    end
end
end


function vx_ego = estimate_vxego_ransac(vr, alpha, num_ransac_trials, ransac_num_samples, inlier_v_thresh)

% peform ransac with ransac_num_samples samples 
best_trial_inds = zeros(ransac_num_samples,1);
best_num_inliers = 0;
for trial_num = 1:num_ransac_trials
    curr_trial_inds = randsample(length(vr), ransac_num_samples);
    curr_vr = vr(curr_trial_inds);
    curr_alpha = alpha(curr_trial_inds);
    curr_vx_ego = - curr_vr / cos(curr_alpha); 
    
    all_vr_curr_trial = - curr_vx_ego * cos(alpha);
    num_inliers = sum(abs(all_vr_curr_trial - vr) < inlier_v_thresh);
    if num_inliers > best_num_inliers
        best_num_inliers = num_inliers;
        best_trial_inds = curr_trial_inds;
    end
end

% determine all inliers
if best_trial_inds ~= 0
    best_vr = vr(best_trial_inds);
    best_alpha = alpha(best_trial_inds);
    best_vx_ego = - best_vr / cos(best_alpha); 
    all_vr_best_trial = - best_vx_ego * cos(alpha);
    inlier_inds = abs(all_vr_best_trial - vr) < inlier_v_thresh;

    % re-estimate vx_ego using the inliers
    inlier_vr = vr(inlier_inds);
    inlier_alpha = alpha(inlier_inds);
    vx_ego = - inlier_vr / cos(inlier_alpha);
else
    vx_ego = Inf;
end

end
                        
