function [obj] = imm_predict(obj,dt) %#codegen
% Modell 1:
% x = [dx,dy,h,omega,s, 0]' 
% Modell 2:
% 6D Kalman predction (linear motion)
% x = [dx vx ax dy vy ay]'

    anz_IMM_models = 3;
    if abs(dt) > 0
        for imm_idx = 1:anz_IMM_models
            [obj.IMM(imm_idx)] = calc_prediction(obj.IMM(imm_idx),imm_idx,obj.standing,dt);
        end
    end
end 

function [obj] = calc_prediction(obj,imm_idx,standing,dt)
    if imm_idx == 1     % Constant turn rate model
        [obj.x, obj.P] = kalman_pre_turn(obj.x,obj.P,dt,standing);
    elseif imm_idx == 2 % Cartesian model high-dyn
        [obj.x, obj.P] = kalman_pre_cart_hd(obj.x,obj.P,dt,standing);
    else                % Cartesian model low-dyn
        [obj.x, obj.P] = kalman_pre_cart_ld(obj.x,obj.P,dt,standing);
    end
end

function [x,P] = kalman_pre_cart_hd(x,P,dt,standing)
% x = [dx,vx,ax,dy,vy,ay]'

    if standing > 0.5
        NF = 0.5^2;
        
        % system matrix
        A = [1   dt   0   0   0    0;
             0   1    0   0   0    0;  
             0   0    0   0   0    0;
             0   0    0   1   dt   0;
             0   0    0   0   1    0;
             0   0    0   0   0    0];
        
        % system noise cov
        Q = NF  *  [1/3*dt^3   1/2*dt^2   0      0   0   0;
                    1/2*dt^2   dt         0      0   0   0;
                    0          0          0      0   0   0;
                    0   0   0                    1/3*dt^3   1/2*dt^2  0;
                    0   0   0                    1/2*dt^2   dt        0;
                    0   0   0                    0          0         0];
    else
        NF = 1^2;

        % system matrix
        A = [1   dt  0.5*dt^2  0   0   0;
             0   1   dt        0   0   0;
             0   0   1         0   0   0;
             0   0   0         1   dt  0.5*dt^2;
             0   0   0         0   1   dt;
             0   0   0         0   0   1];
        
        % system noise cov
        Q = NF  *  [1/20*dt^5   1/8*dt^4   1/6*dt^3   0   0   0;
                    1/8*dt^4    1/3*dt^3   1/2*dt^2   0   0   0;
                    1/6*dt^3    1/2*dt^2   dt         0   0   0;
                    0   0   0                         1/20*dt^5   1/8*dt^4   1/6*dt^3;
                    0   0   0                         1/8*dt^4    1/3*dt^3   1/2*dt^2;
                    0   0   0                         1/6*dt^3    1/2*dt^2   dt];
    end
    
    % state prediction
    x = A*x;
    
    % cov prediction
    if dt > 0 % Kein Cov-Prediktion bei Retrodiktion
        P = A*P*A' + Q; 
    end
    
end

function [x,P] = kalman_pre_cart_ld(x,P,dt,standing)
% x = [dx,vx,ax,dy,vy,ay]'
       
    if standing > 0.5
        NF = 0.25^2;
        
        % system matrix
        A = [1   dt   0   0   0    0;
             0   1    0   0   0    0;  
             0   0    0   0   0    0;
             0   0    0   1   dt   0;
             0   0    0   0   1    0;
             0   0    0   0   0    0];
        
        % system noise cov
        Q = NF  *  [1/3*dt^3   1/2*dt^2   0      0   0   0;
                    1/2*dt^2   dt         0      0   0   0;
                    0          0          0      0   0   0;
                    0   0   0                    1/3*dt^3   1/2*dt^2  0;
                    0   0   0                    1/2*dt^2   dt        0;
                    0   0   0                    0          0         0];
    else
        NF = 0.5^2;
        
        % system matrix
        A = [1   dt   0   0   0    0;
             0   1    0   0   0    0;
             0   0    0   0   0    0;
             0   0    0   1   dt   0;
             0   0    0   0   1    0;
             0   0    0   0   0    0];
        
        % system noise cov
        Q = NF  *  [1/3*dt^3   1/2*dt^2   0      0   0   0;
                    1/2*dt^2   dt         0      0   0   0;
                    0          0          0      0   0   0;
                    0   0   0                    1/3*dt^3   1/2*dt^2  0;
                    0   0   0                    1/2*dt^2   dt        0;
                    0   0   0                    0          0         0];
    end

    % state-prediction
    x = A*x;
    
    % cov-prediction
    if dt > 0
        P = A*P*A' + Q; 
    end
    
end




function [x,P] = kalman_pre_turn(x,P,dt,standing)
% x = [dx,dy,h,omega,s, 0]'
     
    h = x(3);    
    omega = x(4); 
    s = x(5);
    
    vx = s*cos(h);
    vy = s*sin(h);

    if abs(omega) > 0
        SW = sin(omega*dt)/(omega*dt);
        CW = (1 - cos(omega*dt))/(omega*dt);
        AW = 1/omega * (cos(omega*dt) - SW);
        BW = 1/omega * (sin(omega*dt) - CW);
    else
        SW = 1;
        CW = 0;
        AW = 0;
        BW = 0.5*dt;
    end
    
    % state-prediction
    if standing > 0.5
        x(1) = x(1) + vx*dt;
        x(2) = x(2) + vy*dt;
        x(3) = h;
        x(4) = 0;
        x(5) = s;
        x(6) = 0;
    else
        x(1) = x(1) + dt*(SW*vx - CW*vy);
        x(2) = x(2) + dt*(CW*vx + SW*vy);
        x(3) = h + omega*dt;
        x(4) = omega;
        x(5) = s;
        x(6) = 0;
    end
    

    % cov-prediction
    if dt > 0
        
        a13 = -dt*(SW*vy + CW*vx);
        a14 = dt*(AW*vx - BW*vy);
        a15 = dt*(SW*cos(h) - CW*sin(h));
        a23 = dt*(SW*vx - CW*vy);
        a24 = dt*(AW*vy + BW*vx);
        a25 = dt*(SW*sin(h) + CW*cos(h)); 

        if standing > 0.5
            var_xy = 0.25^2; 
            var_omega = 0.1^2;
            var_s = 0.5^2; 
            var_h = 0.1^2;
        else
            var_xy = 0.25^2; 
            var_omega = 0.5^2;
            var_s = 1^2;
            var_h = 0.1^2;         
        end
        A = [1  0   a13      a14   a15  0;
             0  1   a23      a24   a25  0;
             0  0   1        dt    0    0;
             0  0   0        1     0    0;
             0  0   0        0     1    0;
             0  0   0        0     0    0];  

        Q= [1/3*dt^3*var_xy 0               0                              0                  0          0;   % 1/3*dt^3*var_xy is for consistency with CV/CA
            0               1/3*dt^3*var_xy 0                              0                  0          0;
            0               0               (dt^3/3*var_omega + dt*var_h)  dt^2/2*var_omega   0          0;
            0               0               dt^2/2*var_omega               dt*var_omega       0          0;
            0               0               0                              0                  dt*var_s   0;
            0               0               0                              0                  0          0]; 
        
        P = A*P*A' + Q;
        
    end
    
end


