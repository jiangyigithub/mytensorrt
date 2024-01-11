function measurement = ekf_6D_CA_state2meas_obj_offset(state_vector, sens_offset_x, sens_offset_y, vx_ego, x_offset_obj, y_offset_obj, obj_orntn) %#codegen
%EKF_6D_CA_STATE2MEAS_OBJ_OFFSET conversion of state vector to measurement
% same as ekf_6D_CA_state2meas, but with x-y offset in object coordinate
% system
% 
% state vector = [px, vx, ax, py, vy, ay].'
% measurement = [range, alpha, v_radial].'

assert(numel(state_vector)==6); % single state vector
% augment state with offset 
offset_x =  x_offset_obj * cos (-obj_orntn) + y_offset_obj * sin(-obj_orntn); % in global coordinate system
offset_y = -x_offset_obj * sin (-obj_orntn) + y_offset_obj * cos(-obj_orntn);

state_augmented = state_vector + [offset_x; 0; 0; offset_y; 0; 0];

% use standard function for measurement 
measurement = ekf_6D_CA_state2meas(state_augmented, sens_offset_x, sens_offset_y, vx_ego);

end

