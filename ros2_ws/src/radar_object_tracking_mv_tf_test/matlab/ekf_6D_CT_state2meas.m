function measurement = ekf_6D_CT_state2meas(state_vector, sens_offset_x, sens_offset_y, vx_ego) %#codegen
%EKF_6D_CT_STATE2MEAS conversion of state vector to measurement
% state vector = [px, py, heading, turn_rate, v_abs].'
% measurement = [range, alpha, v_radial].'

px_sensor = state_vector(1) - sens_offset_x;  % x-position in sensor coordinate system
py_sensor = state_vector(2) - sens_offset_y; 

range = sqrt(px_sensor.^2 + py_sensor.^2);

% vx=v_abs*cos(heading); vy=v_abs*sin(heading)
vx_sensor = state_vector(5)*cos(state_vector(3)) - vx_ego;  % x-velocity in sensor coordinate system
vy_sensor = state_vector(5)*sin(state_vector(3));           % y-velocity in sensor coordinate system

v_radial = (vx_sensor.*px_sensor + vy_sensor.*py_sensor)./range;  % v_r = v_vec.'p_vec/norm(p_vec)

alpha = atan2(py_sensor, px_sensor);

measurement = [range; alpha; v_radial];

end

