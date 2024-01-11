function [obj, y_aso, is_associated, mark_as_associated] = asso_gating(obj,y, sensor_pos_offset, vx_ego) %#codegen
% Gating with fixed window-size
% y = [dr,alpha,vr]'
% x = [dx vx ax dy vy ay]'

coder.inline('never');

y_aso = [NaN NaN NaN]';
is_associated = false;
mark_as_associated = false;

obj_range = sqrt(obj.x(1).^2 + obj.x(4)^2);
obj_angle = atan2(obj.x(4),obj.x(1)); % only rough angle estimate to avoid 2pi jump, not including sensor position offset


range_gate_abs = max(y(1)/20, 4);
angle_gate_abs = 2 *pi/180; % 2 degree
vr_absGate = max(max(obj_range/10, abs(y(3)*0.20)), 2);
if y(1) < 15 % ignoring elevation makes a difference for near objects
    vr_absGate = vr_absGate + 0.5*abs(vx_ego) + 0.2*abs(y(3));
    angle_gate_abs = (17-y(1)) *pi/180; % 2 degree at r=15, 15 degree at r=2
end

% worst case gating:
% check four corner points of object
measurement1 = ekf_6D_CA_state2meas_obj_offset(obj.x, sensor_pos_offset(1), sensor_pos_offset(2), vx_ego, ...
    +obj.length/2, +obj.width/2, obj.psi);
measurement2 = ekf_6D_CA_state2meas_obj_offset(obj.x, sensor_pos_offset(1), sensor_pos_offset(2), vx_ego, ...
    +obj.length/2, -obj.width/2, obj.psi);
measurement3 = ekf_6D_CA_state2meas_obj_offset(obj.x, sensor_pos_offset(1), sensor_pos_offset(2), vx_ego, ...
    -obj.length/2, +obj.width/2, obj.psi);
measurement4 = ekf_6D_CA_state2meas_obj_offset(obj.x, sensor_pos_offset(1), sensor_pos_offset(2), vx_ego, ...
    -obj.length/2, -obj.width/2, obj.psi);

% range interval
corner_ranges = [measurement1(1), measurement2(1), measurement3(1), measurement4(1)];
range_max = max(corner_ranges) + range_gate_abs;
range_min = max(min(corner_ranges) - range_gate_abs, 0);
obj.asso.range_interval = range_max - range_min;

% angle interval
corner_angles = [measurement1(2), measurement2(2), measurement3(2), measurement4(2)];
corner_angles_obj_cosy = wrapToPi_c(corner_angles - obj_angle); % transform to obj to avoid 2pi ambiguity
angle_max = max(corner_angles_obj_cosy) + angle_gate_abs;
angle_min = min(corner_angles_obj_cosy) - angle_gate_abs;
obj.asso.ang_interval = angle_max - angle_min;

% velocity interval
corner_radial_velocities = [measurement1(3), measurement2(3), measurement3(3), measurement4(3)];
v_radial_min = min(corner_radial_velocities) - vr_absGate;
v_radial_max = max(corner_radial_velocities) + vr_absGate;
obj.asso.vel_interval = v_radial_max - v_radial_min;

% range Gating
if (y(1) < range_max) && (y(1) > range_min)
    % angle Gating
    meas_ang_obj_cosy = wrapToPi_c(y(2) - obj_angle);
    if (meas_ang_obj_cosy > angle_min) && (meas_ang_obj_cosy < angle_max)
        % vr-Gating
        % velocity differs for parts of car, when vehicle is nearby (e.g. crossing in front of us)
        if  (y(3) > v_radial_min) && (y(3) < v_radial_max)
            y_aso = y;
            is_associated = true;
            mark_as_associated = true;
        elseif abs(y(3)) < vr_absGate
            % probably a wheel reflex from floor
            % not associate, but mark (not generate new objects from it)
            mark_as_associated = true;
        elseif (y(3) > 2*v_radial_min) && (y(3) < 2*v_radial_max)
            % probably a wheel reflex from top (twice speed)
            % not associate, but mark (not generate new objects from it)
            mark_as_associated = true;
        end
    end
end
end


