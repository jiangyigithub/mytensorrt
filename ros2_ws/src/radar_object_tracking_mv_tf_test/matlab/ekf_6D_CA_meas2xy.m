function [px, py] = ekf_6D_CA_meas2xy(range, alpha, px_sens_offset, py_sens_offset) %#codegen
%EKF_6D_CA_MEAS2XY: Conversion of measurement to x-y position 
% (used for gating etc)

px_sens = range * cos(alpha);
py_sens = range * sin(alpha);

px = px_sens + px_sens_offset;
py = py_sens + py_sens_offset;

end

