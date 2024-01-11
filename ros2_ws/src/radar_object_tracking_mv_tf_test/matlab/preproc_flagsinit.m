function [obj] = preproc_flagsinit(obj, meas_list, dt) %#codegen


obj.meas = false;
obj.hist = true;
obj.meas_angles = 0;

% Is Object in sensor FOV? -> obj.measureable
obj_px_no_offset = obj.x(1) - meas_list.dx_sens_offset; % still in vehicle CoSy orientation
obj_py_no_offset = obj.x(4) - meas_list.dy_sens_offset;


obj_ang_no_offset = atan2(obj_py_no_offset,obj_px_no_offset); % 4-quadrant tan (atan2) already considers pi/2 ambiguity of tan; value in [-pi,pi] 

obj_r_sens = sqrt( obj_px_no_offset^2 + (obj_py_no_offset)^2);
obj_ang_sens = obj_ang_no_offset - meas_list.ang_sens_offset;
if obj_ang_sens < -pi
    obj_ang_sens = obj_ang_sens + 2*pi; 
elseif obj_ang_sens > pi
    obj_ang_sens = obj_ang_sens - 2*pi;
end

if (obj_ang_sens<meas_list.fov_angle(1)) || (obj_ang_sens>meas_list.fov_angle(end))
    % Object out of FOV bounds
    obj.measureable = false;
else
    fov_sec_index_lower = find(obj_ang_sens > meas_list.fov_angle,1,'last');
    fov_sec_index_higher = find(obj_ang_sens < meas_list.fov_angle,1,'first');
    
%     fov_r_interp = interpolate_2points(meas_list.fov_angle(fov_sec_index_lower), ...
%                                        meas_list.fov_angle(fov_sec_index_higher), ...
%                                        meas_list.fov_range(fov_sec_index_lower), ...
%                                        meas_list.fov_range(fov_sec_index_higher), ...
%                                        obj_ang_sens);

    fov_r_interp = max(meas_list.fov_range(fov_sec_index_lower), meas_list.fov_range(fov_sec_index_higher));
    
    if obj_r_sens < fov_r_interp
        obj.measureable = true;
    else
        obj.measureable = false;
    end
end

anz_IMM_models = 3;
for i = 1:anz_IMM_models
    obj.IMM(i).pdf_max = 0;
end

if dt > 0
    obj.age = obj.age + dt;
end

% update timestamp
obj.t_abs = meas_list.t; 

end

function y = interpolate_2points(x0,x1,y0,y1, x) 
  y = y0 + (y1-y0)*(x-x0)/(x1-x0);
end



