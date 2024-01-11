function jacobian = ekf_6D_CA_getJacobian(state_vector) %#codegen
%EKF_6D_CA_GETJACOBIAN get Jacobian for EKF update 
% derivative of measurement for state vector (delta y)/(delta x)
% !!! (px, py) must be already adjusted for sensor position and grid offset
% 
% state vector = [px, vx, ax, py, vy, ay].' 
% measurement = [range, alpha, v_radial].'

% r = sqrt(px^2 + py^2)
% alpha = atan2( py/px )
% vr = (px*vx + py*vy) / r

range = sqrt(state_vector(1).^2 + state_vector(4).^2);

% delta_r_delta_px = px / r
delta_r_delta_px = state_vector(1) / range;
% delta_r_delta_py = py / r
delta_r_delta_py = state_vector(4) / range;

% delta_alpha_delta_px = -py/r^2
delta_alpha_delta_px = -state_vector(4) / range^2;
% delta_alpha_delta_py = px/r^2
delta_alpha_delta_py = state_vector(1) / range^2;

% delta_vr_delta_px = vx/r - px*(px*vx + py*vy)/r^3
delta_vr_delta_px = state_vector(2)/range - state_vector(1)*(state_vector(1)*state_vector(2) + state_vector(4)*state_vector(5))/range^3;
% delta_vr_delta_py = vy/r - py*(px*vx + py*vy)/r^3
delta_vr_delta_py = state_vector(5)/range - state_vector(4)*(state_vector(1)*state_vector(2) + state_vector(4)*state_vector(5))/range^3;
% delta_vr_delta_vx = px/r
delta_vr_delta_vx = state_vector(1)/range;
% delta_vr_delta_vy = py/r
delta_vr_delta_vy = state_vector(4)/range;

jacobian = [delta_r_delta_px, 0, 0, delta_r_delta_py, 0, 0; ...
    delta_alpha_delta_px, 0, 0, delta_alpha_delta_py, 0 , 0;...
    delta_vr_delta_px, delta_vr_delta_vx, 0, delta_vr_delta_py, delta_vr_delta_vy, 0];

end

