function jacobian = ekf_6D_CT_getJacobian(state_vector) %#codegen
%EKF_6D_CA_GETJACOBIAN get Jacobian for EKF update 
% derivative of measurement for state vector (delta y)/(delta x)
% state vector = [px, py, heading, turn_rate, v_abs, 0].'
% measurement = [range, alpha, v_radial].'

% r = sqrt(px^2 + py^2)
% alpha = atan2( py/px )
% vr = (px*vx + py*vy) / r
% vx = v_abs * cos(heading)
% vy = v_abs * sin(heading)

range = sqrt(state_vector(1).^2 + state_vector(2).^2);

% delta_r_delta_px = px / r
delta_r_delta_px = state_vector(1) / range;
% delta_r_delta_py = py / r
delta_r_delta_py = state_vector(2) / range;

% delta_alpha_delta_px = -py/r^2
delta_alpha_delta_px = -state_vector(2) / range^2;
% delta_alpha_delta_py = px/r^2
delta_alpha_delta_py = state_vector(1) / range^2;

% delta_vr_delta_px = vs*cos(psi)/r - vs*cos(psi)*px^2/r^3
delta_vr_delta_px = state_vector(5)*cos(state_vector(3))/range - state_vector(5)*cos(state_vector(3))*state_vector(1)^2/range^3;
% delta_vr_delta_py = vs*cos(psi)/r - vs*cos(psi)*py^2/r^3
delta_vr_delta_py = state_vector(5)*cos(state_vector(3))/range - state_vector(5)*cos(state_vector(3))*state_vector(2)^2/range^3;
% delta_vr_delta_psi = (-sin(psi)*vs*px + cos(psi)*vs*py) / r
delta_vr_delta_psi = state_vector(5)*(-sin(state_vector(3))*state_vector(1) + cos(state_vector(3))*state_vector(2)) / range;
% delta_vr_delta_vs = (px*cos(psi + py*sin(psi)) / r
delta_vr_delta_vs = (state_vector(1)*cos(state_vector(3)) + state_vector(2)*sin(state_vector(3))) / range;

jacobian = [delta_r_delta_px, delta_r_delta_py, 0, 0, 0, 0;...
    delta_alpha_delta_px, delta_alpha_delta_py, 0, 0, 0, 0;...
    delta_vr_delta_px, delta_vr_delta_py, delta_vr_delta_psi, 0, delta_vr_delta_vs, 0];

end

