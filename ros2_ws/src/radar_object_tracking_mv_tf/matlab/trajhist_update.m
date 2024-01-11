function [obj_list] = trajhist_update(obj_list) %#codegen
coder.extrinsic('disp')

BUF_LEN_MAX = 20;



for obj_idx = 1:length(obj_list)
    if (obj_list(obj_idx).valid == true) ... % && (obj_list(obj_idx).meas == true) ...
            && ((obj_list(obj_idx).moving > 0.95) || (obj_list(obj_idx).traj_hist_len > 0)) % always update if once updated
        
        % Start with smaller distances, to have quickly a full buffer */
        if obj_list(obj_idx).traj_hist_len < 5
            DELTA_BUF = 1;
        elseif obj_list(obj_idx).traj_hist_len < 10
            DELTA_BUF = 2;
        else
            vabs = sqrt(obj_list(obj_idx).x(2)^2 + obj_list(obj_idx).x(5)^2);
            DELTA_BUF = max(2, 0.2*vabs);
        end
                
        % center of rear axis
        dx_rear = -0.5*obj_list(obj_idx).length*cos(obj_list(obj_idx).psi_traj_hist)*0;
        dy_rear = -0.5*obj_list(obj_idx).length*sin(obj_list(obj_idx).psi_traj_hist)*0;  
        
        if obj_list(obj_idx).traj_hist_len == 0
            obj_list(obj_idx).traj_hist_x(1)  = obj_list(obj_idx).x(1) + dx_rear; % + obj_list(obj_idx).x_refl(1);
            obj_list(obj_idx).traj_hist_y(1)  = obj_list(obj_idx).x(4) + dy_rear; % + obj_list(obj_idx).y_refl(1);            
            obj_list(obj_idx).traj_hist_vx(1) = obj_list(obj_idx).x(2);
            obj_list(obj_idx).traj_hist_vy(1) = obj_list(obj_idx).x(5);
            obj_list(obj_idx).traj_hist_t(1)  = obj_list(obj_idx).t_abs;
            
            obj_list(obj_idx).traj_hist_len = 1;
            
        else
            
            delta_dx = obj_list(obj_idx).x(1) + dx_rear - obj_list(obj_idx).traj_hist_x(1);
            delta_dy = obj_list(obj_idx).x(4)+ dy_rear - obj_list(obj_idx).traj_hist_y(1);

            delta_abs = (delta_dx^2 + delta_dy^2);

            if (delta_abs > (DELTA_BUF^2))  % minimum distance traveled or new object                
                obj_list(obj_idx).traj_hist_x  = [obj_list(obj_idx).x(1) + dx_rear, obj_list(obj_idx).traj_hist_x(1:end-1)]; % + obj_list(obj_idx).x_refl(1);
                obj_list(obj_idx).traj_hist_y  = [obj_list(obj_idx).x(4) + dy_rear, obj_list(obj_idx).traj_hist_y(1:end-1)];% + obj_list(obj_idx).y_refl(1);
                obj_list(obj_idx).traj_hist_vx = [obj_list(obj_idx).x(2), obj_list(obj_idx).traj_hist_vx(1:end-1)];
                obj_list(obj_idx).traj_hist_vy = [obj_list(obj_idx).x(5), obj_list(obj_idx).traj_hist_vy(1:end-1)];            
                obj_list(obj_idx).traj_hist_vx = [obj_list(obj_idx).t_abs, obj_list(obj_idx).traj_hist_vx(1:end-1)];

                obj_list(obj_idx).traj_hist_len = min(BUF_LEN_MAX,(obj_list(obj_idx).traj_hist_len + 1));

            end  
        end
        
    end
end





