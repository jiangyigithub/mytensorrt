function [obj] = get_clean_obj() %#codegen


anz_IMM_models = 3;

obj.valid = false;
obj.meas  = false;
obj.meas_angles = 0; % counter for associated angles
obj.measureable = false;
obj.hist  = false;
obj.age  = 0;
obj.t_abs = 0;
obj.t_lastmeas = 0;
obj.standing  = 0; % probability
obj.moving  = 0; % probability
obj.pexist  = 0; % probability
obj.Pgr_reflex = 0;   % Guardrail Reflex
obj.x = zeros(6,1);
obj.P = zeros(6,6);
obj.length = 0;
obj.width = 0;
obj.psiDt = 0;
obj.psi = 0;
obj.P_psi = zeros(2,2);
obj.RCS_filt = 0;
obj.P_obj_type = zeros(1,2);

obj.length_meas = 0;
obj.length_filt = 0;
obj.width_meas = 0;
obj.width_filt = 0; 

obj.traj_hist_x = zeros(1,20);      % historical trajectory (world coordinates)
obj.traj_hist_y = zeros(1,20);
obj.traj_hist_vx = zeros(1,20);
obj.traj_hist_vy = zeros(1,20);
obj.traj_hist_t = zeros(1,20);
obj.traj_hist_len = 0;
obj.psi_traj_hist = 0;

grid_step = 0.5;
x_cells = -5:grid_step:5;
y_cells = -5:grid_step:5;
len_x_cells = length(x_cells);
len_y_cells = length(y_cells);

grid = struct('step',grid_step,'x_cells',x_cells,'y_cells',y_cells,'x_len',len_x_cells,...
              'y_len',len_y_cells,'LR',zeros(len_x_cells,len_y_cells));
obj.grid = grid;

asso = struct('range_interval',0,'vel_interval',0,'ang_interval',0);
obj.asso = asso;

IMM = struct('x',zeros(6,1),'P',zeros(6,6),'mu',1/anz_IMM_models,'pdf_max',0);
IMM_array = repmat(IMM,1,anz_IMM_models);

obj.IMM = IMM_array;

coder.cstructname(obj, 'OBJECT_STRUCT');

