function [obj] = update_derived_values(obj,measurements,asso_list) %#codegen
% update additional state attributes (State UPdate)
% - orientation and yaw rate
% - object extent (width/length)
% - classificationof moving object

    
% calculate object moving direction from historical trajectory ---------------------------
if (obj.traj_hist_len > 5)
    
    % middle of rear axis
    dx_rear = -0.5*obj.length*cos(obj.psi_traj_hist)*0;
    dy_rear = -0.5*obj.length*sin(obj.psi_traj_hist)*0; 
    
    % transform object to middle of rear axis
    dx_curent = obj.x(1) + dx_rear; % + obj.x_refl(1);
    dy_curent = obj.x(4) + dy_rear; % + obj.y_refl(1);

    % get last position for rough orientation
    delta_dx = obj.traj_hist_x(1) - obj.traj_hist_x(2);
    delta_dy = obj.traj_hist_y(1) - obj.traj_hist_y(2);
    % rough estimate of orientation
    psi0 = atan2(delta_dy,delta_dx);

    % add current position to trajectory-buffer
    buffer_x0 = zeros(1,11);
    buffer_y0 = zeros(1,11);
    buffer_idx_max = min(obj.traj_hist_len,4);
    
    buffer_x0(1:buffer_idx_max) = obj.traj_hist_x(1:buffer_idx_max);
    buffer_y0(1:buffer_idx_max) = obj.traj_hist_y(1:buffer_idx_max);

    % rotate to rough orientation, to avoid pole at 90 degree
    buffer_x_trans =  buffer_x0*cos(psi0) + buffer_y0*sin(psi0);
    buffer_y_trans = -buffer_x0*sin(psi0) + buffer_y0*cos(psi0);

    if (obj.traj_hist_len >= 5)
        % parabola-fit
        coef = polyfit(buffer_x_trans(1:(1+buffer_idx_max)),buffer_y_trans(1:(1+buffer_idx_max)),2);
        % get slope at current position
        slope = 2*coef(1)*(dx_curent*cos(psi0) + dy_curent*sin(psi0)) + coef(2);
    else
        % line-fit
        coef = [polyfit(buffer_x_trans(1:(1+buffer_idx_max)),buffer_y_trans(1:(1+buffer_idx_max)),1) 0];
        slope = coef(1);
    end
    
    % calculate orientation
    psi = atan(slope) + psi0;

    % limit to +/- pi
    if psi > pi
        psi = psi - 2*pi;
    elseif psi < -pi
        psi = psi + 2*pi;
    end
    obj.psi_traj_hist = psi;
end
    
% output orientation
v_abs = sqrt(obj.x(2)^2 + obj.x(5)^2);
if (v_abs < 1.0)
    if (obj.traj_hist_len > 5)
        obj.psi = obj.psi_traj_hist;
    else
        %obj.psi = obj.IMM(1).x(3);          % use the CTR state
        obj.psi = atan2(obj.x(5),obj.x(2));  % use the object velocity
    end
else
    %obj.psi = obj.IMM(1).x(3);         % use the CTR state
    obj.psi = atan2(obj.x(5),obj.x(2)); % use the object velocity
end

% extent (length/width) estimation -------------------------------------------------------------
if obj.meas == 1    % new information only through measurements (moving classification)
    alpha = atan2(obj.x(4),obj.x(1));
    delta_vr_Standing = abs(cos(alpha)*obj.x(2) + sin(alpha)*obj.x(5)); % radial velovity over ground
    if (obj.moving == 1.0) && ((obj.traj_hist_len >= 5) || (delta_vr_Standing > 2)) % orientation reliable?
        prob_grid = obj.grid.LR ./ (1 + obj.grid.LR);  % convert odds to prob
        prob_grid_x = sum(prob_grid - min(prob_grid(:)), 2);
        accu_grid_x = cumsum(prob_grid_x);
        accu_grid_x = accu_grid_x/max(accu_grid_x);
        lr_x_begin = find_first(accu_grid_x > 0.3);
        lr_x_end = find_first(accu_grid_x > 0.7);
        prob_grid_y = sum(prob_grid - min(prob_grid(:)), 1);
        accu_grid_y = cumsum(prob_grid_y);
        accu_grid_y = accu_grid_y/max(accu_grid_y);
        lr_y_begin = find_first(accu_grid_y > 0.3);
        lr_y_end = find_first(accu_grid_y > 0.7);

        obj.length_meas = lr_x_end - lr_x_begin + obj.grid.step;
        obj.width_meas = lr_y_end - lr_y_begin + obj.grid.step;
        
        obj.length_filt = 0.9 * obj.length_filt + 0.1 * obj.length_meas;
        obj.width_filt = 0.9 * obj.width_filt + 0.1 * obj.width_meas;

        obj.length = max(obj.length_filt,1);
        obj.width = max(obj.width_filt,1);
    end
    
    
    % filter RCS ------------------------------------------------------------------
    % search location with strongest RCS
    max_rcs = -128;
    for asso_idx = 1:length(asso_list)
        if (asso_list(asso_idx).loc_nr > 0)
            idx_meas = asso_list(asso_idx).loc_nr;
            anz_rcs = 0;
            sum_rcs = 0;
            for rcs_idx = 1:length(measurements.dBRcs(:,idx_meas))
                rcs_value = measurements.dBRcs(rcs_idx,idx_meas);
                if rcs_value < -127
                    continue;   % invalid RCS
                else
                    sum_rcs = sum_rcs + rcs_value;
                    anz_rcs = anz_rcs + 1;
                end
            end
            mean_rcs = sum_rcs/anz_rcs;
            max_rcs = max(max_rcs, mean_rcs);
        end
    end
    % RCS low-pass filter
    if max_rcs > -128
        obj.RCS_filt = 0.9*obj.RCS_filt + 0.1*max_rcs;
    end

    
    % Moving classification ----------------------------------------------------------
    alpha = atan2(obj.x(4),obj.x(1));
    delta_vr_Standing = abs(cos(alpha)*obj.x(2) + sin(alpha)*obj.x(5)); % radial velocity
    delta_vr_Standing2 = sqrt(obj.x(2)^2 + obj.x(5)^2); % velocity over ground

    w_vrel = abs(sin(alpha));
    delta_vr_Standing = w_vrel*delta_vr_Standing + (1-w_vrel)*delta_vr_Standing2; % weighting: front=v_ground, side=v_radial

    if (obj.moving < 0.99) || (obj.age < 1) % "lock" at high velocity
        % transition probability
        P_ij = [ 0.90         0.10       % old hypothesis 1: not mobile
                 0.05         0.95];     % old hypothesis 2: mobile
            % new not mobile | new mobile

        % calculation of a-priori hypothesis probability
        C_nmobil  = P_ij(1,1)*(1 - obj.moving) + P_ij(2,1)*obj.moving;
        C_mobil   = P_ij(1,2)*(1 - obj.moving) + P_ij(2,2)*obj.moving;

        % calculation of current hypothesis probabilities
        varvabs = 0.5^2;
        p_nmobil = 1/sqrt(2*pi*varvabs) * exp(-0.5*delta_vr_Standing^2/varvabs);
        p_mobil  = 0.1;    % uniform distribution  1/(10 m/s)

        % calculation of the a-posteriori hypothesis probabilities
        obj.moving = C_mobil*p_mobil/(C_mobil*p_mobil + C_nmobil*p_nmobil);
    else
        obj.moving = 1;   % mobile once => always mobile
    end
    obj.standing = 1 - obj.moving;

end
end
% -----------------------------------------------------------------

function last_idx = find_last(array_of_bool)

last_idx = 0;
for i = length(array_of_bool):-1:1
    if array_of_bool(i)
        last_idx = i;
        return
    end
end

end

function first_idx = find_first(array_of_bool)

first_idx = 0;
for i = 1:length(array_of_bool)
    if array_of_bool(i)
        first_idx = i;
        return
    end
end

end

