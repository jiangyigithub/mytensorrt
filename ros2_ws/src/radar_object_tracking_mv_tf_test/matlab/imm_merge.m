function [obj] = imm_merge(obj, vx_ego) %#codegen
% IMM Merge: estimate joint state vector for data association
%
% Input:
% IMM: 
%   IMM(i).x: state vector of model i
%   IMM(i).P: covariance of model i
%   IMM(i).mu: Probability for model i
%
% Output:
% x: Zusammengef�hrter Zustand
% P: Zusammengef�hrte Kovarianz
%
num_IMM_models = 3;

% Shortcut for IMM-structure
IMM = obj.IMM;

% transform model 1 from constant-turn-rate to Cartesian
[IMM(1).x, IMM(1).P] = turn2cartesian(IMM(1).x, IMM(1).P, vx_ego);

% merge state vectors
x = zeros(size(IMM(1).x));    
for i = 1:num_IMM_models
    x = x + IMM(i).mu * IMM(i).x;   
end

% merge covariances
P = zeros(size(IMM(1).P));  
for i = 1:num_IMM_models
    P = P + IMM(i).mu * IMM(i).P;   
end

% set output
obj.P = P;
obj.x = x;

