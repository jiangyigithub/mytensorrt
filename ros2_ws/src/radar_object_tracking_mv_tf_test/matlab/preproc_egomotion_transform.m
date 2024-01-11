function [ obj ] = preproc_egomotion_transform( obj,meas_list,dt) %#codegen
% shift the objects by the ego_motion*dt, because tracking is in ego-local
% CoSy. 

    anz_IMM_models = 3;

    % transform IMMs
    for imm_idx = 1:anz_IMM_models
        [obj.IMM(imm_idx)] = calc_egomotion_transform(obj.IMM(imm_idx),meas_list.vx_ego,meas_list.ax_ego,meas_list.vx_ego*sin(meas_list.beta_ego),meas_list.psiDt_ego,dt,imm_idx);
    end
    
    % transform fused state
    [obj] = calc_egomotion_transform(obj,meas_list.vx_ego,meas_list.ax_ego,meas_list.vx_ego*sin(meas_list.beta_ego),meas_list.psiDt_ego,dt, 0);
    
end % end function

% ------------------------------------------------------------------------------------------------------------

% Perform coordinate transform
function [obj] = calc_egomotion_transform( obj,vx_ego,ax_ego,vy_ego,psiDt_ego,dt, imm_idx)

    % TODO: also update vy_ego and ax_ego. 
    
    if abs(psiDt_ego) > 0.0001
        delta_phi = psiDt_ego*dt;
        delta_x   = vx_ego/psiDt_ego * sin(delta_phi);
        delta_y   = vx_ego/psiDt_ego * (1 - cos(delta_phi));
    else
        delta_x  = vx_ego*dt;
        delta_y  = 0;
        delta_phi = 0;
    end
    
    if imm_idx == 1
        % translation 
        x_abs = obj.x - [delta_x  delta_y  0  0  0  0]';
        P_abs = obj.P;  % do not change covariance
        
        % define rotation matrix
        T2 = [ cos(delta_phi)  sin(delta_phi);
              -sin(delta_phi)  cos(delta_phi)];
        
        % perform rotation
        x_abs(1:2,1)   = T2*x_abs(1:2,1);
        P_abs(1:2,1:2) = T2*P_abs(1:2,1:2)*T2';
        x_abs(3)       = x_abs(3) - delta_phi; % psi, heading

        obj.x = x_abs;
        obj.P = P_abs;
    else       
        % Update location (ego-motion compensation)
        x_abs = obj.x - [delta_x  0  0  delta_y  0  0]';

        % define rotation matrix
        Tsin = eye(3,3)*sin(delta_phi);
        Tcos = eye(3,3)*cos(delta_phi);
        T = [ Tcos  Tsin;
             -Tsin  Tcos];

        % perform rotation
        x_abs = T*x_abs;
        P_abs = T*obj.P*T';

        obj.x = x_abs;
        obj.P = P_abs;
    end

    % transform yaw angle
    if isfield(obj,'psi')
        
        obj.psi = obj.psi - delta_phi;    
        % limit to +/- pi
        if obj.psi > pi
            obj.psi = obj.psi - 2*pi;
        elseif obj.psi < -pi
            obj.psi = obj.psi + 2*pi;
        end
        
        obj.psi_traj_hist = obj.psi_traj_hist - delta_phi;
        % limit to +/- pi
        if obj.psi_traj_hist > pi
            obj.psi_traj_hist = obj.psi_traj_hist - 2*pi;
        elseif obj.psi_traj_hist < -pi
            obj.psi_traj_hist = obj.psi_traj_hist + 2*pi;
        end
        
    end
    
    % transform historical trajectories
    if isfield(obj,'traj_hist_len')
        if (obj.traj_hist_len > 0) 
            % translation
            obj.traj_hist_x = obj.traj_hist_x - delta_x; 
            obj.traj_hist_y = obj.traj_hist_y - delta_y; 
            % rotation
            traj_hist_x_rot =  cos(delta_phi)*obj.traj_hist_x + sin(delta_phi)*obj.traj_hist_y;
            traj_hist_y_rot = -sin(delta_phi)*obj.traj_hist_x + cos(delta_phi)*obj.traj_hist_y;
            traj_hist_vx_rot =  cos(delta_phi)*obj.traj_hist_vx + sin(delta_phi)*obj.traj_hist_vy;
            traj_hist_vy_rot = -sin(delta_phi)*obj.traj_hist_vx + cos(delta_phi)*obj.traj_hist_vy;
            % write back historical trajectory
            obj.traj_hist_x = traj_hist_x_rot;
            obj.traj_hist_y = traj_hist_y_rot;
            obj.traj_hist_vx = traj_hist_vx_rot;
            obj.traj_hist_vy = traj_hist_vy_rot;
            % trajectory remains in ego-local coordinate system
        end
    end


end


