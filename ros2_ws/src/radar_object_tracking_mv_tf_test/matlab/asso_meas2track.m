function [obj, measurements, asso_list] = asso_meas2track(obj, measurements, idx_meas, asso_list) %#codegen
% measurement association
% - Gating (fixed gate size)

coder.inline('never');

% load measurement ---------------------------------------------------------
dr = measurements.dr(idx_meas);
vr = measurements.vr(idx_meas);
alpha = measurements.alpha(idx_meas);

y = [dr; alpha; vr]; % original measurement space

% gating -------------------------------------------------------------------
sensor_pos_offset = [measurements.dx_sens_offset; measurements.dy_sens_offset];
[obj, y_gated, is_associated, mark_as_associated] = asso_gating(obj,y, sensor_pos_offset, measurements.vx_ego);

if mark_as_associated || is_associated
    measurements.asso(idx_meas) = 1;  
end

if not(is_associated)
    % gating failed
    return;
end

% measurement variance
Rm = [measurements.drVar(idx_meas)   0                                   0;
    0                               measurements.alpVar(idx_meas)        0;
    0                               0                                   measurements.vrVar(idx_meas)];

R = [1.5^2  0  0; 0  (2/180*pi)^2  0; 0  0  0.4^2];
R = max(R,Rm);

obj.meas = true;
obj.t_lastmeas = measurements.t;
obj.meas_angles = obj.meas_angles + 1; % number of associated angles

% copy measuremet to association list
for asso_idx = 1:length(asso_list)
    if asso_list(asso_idx).loc_nr == 0  % free entry
        asso_list(asso_idx).y = y_gated(1:3,1);
        asso_list(asso_idx).R = R(1:3,1:3);
        asso_list(asso_idx).PDH1 = measurements.PDH1(idx_meas);
        asso_list(asso_idx).PDH0 = measurements.PDH0(idx_meas);
        asso_list(asso_idx).potGhost = measurements.potGhost(idx_meas);
        asso_list(asso_idx).loc_nr = idx_meas;

        break;
    end
end
% if list is full, do not use measurement

end