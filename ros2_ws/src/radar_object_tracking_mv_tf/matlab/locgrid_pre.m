function obj = locgrid_pre(obj, dt)
%LOCGRID_PRE is the predict step of the local grd map.
% This is basically only a forgetting functionality of the LR intesity

dr = sqrt(obj.x(1)^2+obj.x(4)^2);

LR_clutter_per_second_per_cell = 0.97;
LR_multipath_per_second_per_cell = min(0.3, 0.3* dr/40);

LR_upd_per_second = LR_clutter_per_second_per_cell * LR_multipath_per_second_per_cell;
LR_upd = LR_upd_per_second^dt;

obj.grid.LR = obj.grid.LR * LR_upd;

% limit upper/lower bound
obj.grid.LR = min(obj.grid.LR, 1e3);  % probability 0.999
obj.grid.LR = max(obj.grid.LR, 1/9);  % probability 0.1

end

