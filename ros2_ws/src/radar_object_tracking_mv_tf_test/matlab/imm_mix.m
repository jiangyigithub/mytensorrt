function [obj] = imm_mix(obj,vx_ego) %#codegen
% IMM Mixer
% 
% Input:
% obj.IMM:

%   IMM(i).x: state vector of model i
%   IMM(i).P: covariance of model i
%   IMM(i).mu: probability for model i
%
% Output:
% obj.IMM: Gemischte Zustands-Vektoren und Kovarianzen
%   IMM(i).x: IMM-mixed state vector of model i
%   IMM(i).P: IMM-mixed covariance of model i
%   IMM(i).mu: probability for model i
%

num_IMM_models = 3;

% P_ij: IMM state transition probabilities (row sum == 1)
P_ij = [0.95    0.025  0.025;   % 1: low-dyn CT  (Prior State)
        0.01   0.98   0.01;   % 2: high-dyn
        0.025   0.025  0.95];  % 3: low-dyn CA
       %  1       2    3       : new state


% Shortcut on IMM-structure
IMM = obj.IMM;

% transform model 1 from constant-turm-rate to Cartesian
[IMM(1).x, IMM(1).P] = turn2cartesian(IMM(1).x, IMM(1).P,vx_ego);


% initialize output structure
IMM_type = struct('x',zeros(6,1),'P',zeros(6,6),'mu',1/num_IMM_models,'pdf_max',0);
IMM_out = repmat(IMM_type,1,num_IMM_models);

% --- apply state transition probabilities --------------------------

% calculate posterior probability of models
for j =  1:num_IMM_models
    IMM_out(j).mu = 0;
    for i = 1:num_IMM_models % IMM_out.mu = P_ij * IMM.mu
        IMM_out(j).mu = IMM_out(j).mu + P_ij(i,j)*IMM(i).mu;    
    end
end


% mu_ij(i,j): conditional probability for model change i->j
mu_ij = zeros(num_IMM_models,num_IMM_models);
for j = 1:num_IMM_models
    for i = 1:num_IMM_models
        mu_ij(i,j) = P_ij(i,j) * IMM(i).mu / IMM_out(j).mu;
    end
end


% --- mix state vectors ---------------------------------------------------
for j = 1:num_IMM_models
    IMM_out(j).x = zeros(6,1);
    for i = 1:num_IMM_models % IMM_out.x = mu_ij * IMM.x
        IMM_out(j).x = IMM_out(j).x + mu_ij(i,j)*IMM(i).x;
    end
end


%--- mix covariances --------------------------------------------------------

% calculate covariance increment
temp_type.Dx_ij = zeros(6,1);  % update state vector difference from i->j
temp_type.DP_ij = zeros(6,6);  % sample covariance of update (Dx_ij) for all i,j
temp = repmat(temp_type,num_IMM_models,num_IMM_models);

for i =  1:num_IMM_models
    for j =  1:num_IMM_models
        temp(i,j).Dx_ij = IMM(i).x - IMM_out(j).x;
    end
end
for i =  1:num_IMM_models
    for j =  1:num_IMM_models
        temp(i,j).DP_ij = temp(i,j).Dx_ij * temp(i,j).Dx_ij';
    end
end

% calculate mixed covariance
for j = 1:num_IMM_models
    IMM_out(j).P = mu_ij(1,j)*( IMM(1).P + temp(1,j).DP_ij );  % i = 1
    for i = 2:num_IMM_models
        IMM_out(j).P = IMM_out(j).P + mu_ij(i,j)*( IMM(i).P + temp(i,j).DP_ij );
    end
end

% transform model 1 from Cartesian to constant turn rate
[IMM_out(1).x, IMM_out(1).P] = cartesian2turn(IMM_out(1).x, IMM_out(1).P);


% set output
obj.IMM = IMM_out;

end







