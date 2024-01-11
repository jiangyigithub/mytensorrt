function meas_reject = asso_quick_reject(obj, meas_list)%#codegen

% discard potential ghost targets
bvReject1 = (meas_list.meas == 0) |	((obj.pexist < 0.8) & (meas_list.potDoubleRefl == 1));		

% Load range meas value
dr = meas_list.dr;

% Quick Reject -----------
% rough range gating
longer_edge = max(obj.length, obj.width);
dr_obj_max = norm([obj.x(1) - meas_list.dx_sens_offset; obj.x(4) - meas_list.dy_sens_offset]) + longer_edge;
dr_obj_min = norm([obj.x(1) - meas_list.dx_sens_offset; obj.x(4) - meas_list.dy_sens_offset]) - longer_edge;
extent_reject_min = dr < (dr_obj_min - 5);
extent_reject_max = dr > (dr_obj_max + 5);

meas_reject = bvReject1 | extent_reject_min | extent_reject_max;

end

