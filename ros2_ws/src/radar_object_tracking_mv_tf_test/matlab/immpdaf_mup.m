function [obj,asso_list] = immpdaf_mup(obj,asso_list, sensor_pos_offset, vx_ego) %#codegen
% IMM mixer
% and Kalman-filter measurement update
%
% calculate pda weights and perform update for each combination of
% associated measurement and cell of the local gridmap individually
%
% y = [dr,alpha,vr]'
% x = [dx vx ax dy vy ay]'
%
% Input:
% IMM: 
%   IMM(i).x: state vector of model i
%   IMM(i).P: state covariance of model i
%   IMM(i).C: posterior probability for model i
%
%
% Output:
% IMM: Struktur
%   IMM(i).mu: probability of model i
%

coder.extrinsic('fprintf');

% array sizes
anz_IMM_models = 3;
Nx = obj.grid.x_len; % number of x-cells in local grid
Ny = obj.grid.y_len; % number of y-cells in local grid
M = length(asso_list); % max number of associated locations

% detection probability parameters
PDH1 = 0.8;
PDH0 = 0.02;
Vc = 5;
BETA0 = PDH0/Vc;


%% calculate update weights
asso_likelihood = zeros(Nx, Ny, M);
for m = 1:M  % bad matlab programming style, but ok for c-code generation
    if asso_list(m).loc_nr==0
        continue
    end
    for nx = 1:Nx
        for ny = 1:Ny
            x_offset_obj = obj.grid.x_cells(nx);
            y_offset_obj = obj.grid.y_cells(ny);
            
            expected_meas_CA = ekf_6D_CA_state2meas_obj_offset(obj.x, ...
                sensor_pos_offset(1), sensor_pos_offset(2), vx_ego, x_offset_obj, y_offset_obj, obj.psi);
            
            curr_meas_residue_CA = asso_list(m).y - expected_meas_CA;
            curr_meas_residue_CA(2) = wrapToPi_c( curr_meas_residue_CA(2) );
            asso_likelihood(nx,ny,m) = exp(-0.5 * (curr_meas_residue_CA.' / asso_list(m).R) * curr_meas_residue_CA); 
        end
    end
end

% gridmap is association prior, to be combined with above likelihood
asso_prior = obj.grid.LR ./ (1 + obj.grid.LR);  % convert odds to prob
asso_posterior = repmat(asso_prior,[1,1,M]) .* asso_likelihood;

% normalize
asso_norm = sum(asso_posterior(:));
if asso_norm == 0
    % association is really bad. 
    obj.meas = false;
    obj.meas_angles = 0;
    asso_list = get_clean_asso_list();
    asso_list = asso_list(1,:); % an empty asso-list element
    return
end
asso_posterior = asso_posterior / asso_norm;

% set small probabilities to zero and renorm 
small_idx = asso_posterior < 0.0005;
asso_posterior(small_idx) = 0.0;
asso_posterior = asso_posterior / sum(asso_posterior(:));


%% update each combination of grid cell and associaited tearget individually.

% model probability update -> see Blackman/Popoli pp 226ff
Lambda = ones(1,anz_IMM_models)*(1-PDH1)*BETA0;

for nx = 1:Nx
    for ny = 1:Ny    
        
        % quick reject low proababilities
        if asso_posterior(nx,ny,:) < 0.0005
            continue;
        end  
        
        x_offset_obj = obj.grid.x_cells(nx);
        y_offset_obj = obj.grid.y_cells(ny);
        % represent local grid map element as state vector
        grid_x_offset_global =  cos(-obj.psi) * x_offset_obj + sin(-obj.psi) * y_offset_obj;
        grid_y_offset_global = -sin(-obj.psi) * x_offset_obj + cos(-obj.psi) * y_offset_obj;
        % state_vector_offset = + grid_offset - sensor_position_offset
        state_vector_offset_linear = [grid_x_offset_global-sensor_pos_offset(1); 0; 0; grid_y_offset_global-sensor_pos_offset(2); 0; 0]; % for IMM(2) and IMM(3)
        state_vector_offset_turnrate = [grid_x_offset_global-sensor_pos_offset(1); grid_y_offset_global-sensor_pos_offset(2); 0; 0 ;0 ;0]; % for IMM(1)
        
        for m = 1:M
            if asso_list(m).loc_nr == 0 % association list entry is empty
                continue
            end
            
            % quick reject low proababilities
            if asso_posterior(nx,ny,m) < 0.0005
                continue;
            end
            
            R = asso_list(m).R;
            y = asso_list(m).y;
            
            for i = 1:anz_IMM_models
                % calculate state Jacobian H and measurement residual e
                if i == 1     
                    % Jacobi-Matrix
                    H = ekf_6D_CT_getJacobian(obj.IMM(i).x + state_vector_offset_turnrate);
                    
                    current_meas_expected = ekf_6D_CT_state2meas_obj_offset(obj.IMM(i).x, ...
                        sensor_pos_offset(1), sensor_pos_offset(2), vx_ego, x_offset_obj, y_offset_obj, obj.psi);
                    % measurement residual vector
                    e = y - current_meas_expected;
                else
                    H = ekf_6D_CA_getJacobian(obj.IMM(i).x + state_vector_offset_linear);
                    current_meas_expected = ekf_6D_CA_state2meas_obj_offset(obj.IMM(i).x, ...
                        sensor_pos_offset(1), sensor_pos_offset(2), vx_ego, x_offset_obj, y_offset_obj, obj.psi);
                    e = y - current_meas_expected;
                end
                
                e(2) = wrapToPi_c( e(2) );
                
                % calculate statistical distance
                S = H*obj.IMM(i).P*H' + R(1:3,1:3);  % measurement residual covariance matrix
                d2 = (e'/S)*e;  % Mahalanobis distance

                pdf = (2*pi)^(-length(S)/2) * det(S)^(-0.5) * exp( -d2/2 );

                % sanity check. Sometimes d2 is too large
                if pdf == Inf || ~isfinite(pdf) || ~isreal(pdf)
                    continue;
                end

                % PDA-formula for likelihood
                Lambda(i) = Lambda(i) + PDH1*pdf; % will be normalized later

                % perform Kalman update
                K = (obj.IMM(i).P*H')/S;
                K = K * asso_posterior(nx,ny,m);  % weight by association probability
                obj.IMM(i).x = obj.IMM(i).x + K*e;
                obj.IMM(i).P = (eye(6,6) - K*H)*obj.IMM(i).P;

                asso_list(m).S(:,:,i) = asso_list(m).S(:,:,i) + S*asso_posterior(nx,ny,m);
                asso_list(m).d2(i) = asso_list(m).d2(i) + d2*asso_posterior(nx,ny,m);
                
            end
        end
    end
end

while obj.IMM(1).x(3) < -pi
    obj.IMM(1).x(3) = obj.IMM(1).x(3) + 2*pi;
end
while obj.IMM(1).x(3) > pi
    obj.IMM(1).x(3) = obj.IMM(1).x(3) - 2*pi;
end


%% update model probabilies
% norming factor -> see Blackman/Popoli p 227
mu_norm = 0;
for i = 1:anz_IMM_models
    mu_norm = mu_norm + Lambda(i)*obj.IMM(i).mu;
end
% calculate model-probability 
for i = 1:anz_IMM_models
    obj.IMM(i).mu = Lambda(i)*obj.IMM(i).mu / mu_norm;
end

