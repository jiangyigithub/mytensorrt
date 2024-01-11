% Global config information
clear variables

c_Config = struct;
% Number of radar locations being used, adapted to Gen5
c_Config.number_locations = 512;
% Below this abs. ego vx, the speed is set t0 0
c_Config.vx_ego_deadzone = 0.05;
% Max. number timestamps to import. Set to sth. suitable in order to safe time during experimenting
c_Config.number_cycles_for_import = Inf;


% Do not change these, only when radar interfaces change
c_Constants = struct;
% bit position of extended measured status of location
c_Constants.LOC_MEASURED = 0;
c_Constants.LOC_MULTITARGETAZI = 1;
c_Constants.LOC_MULTITARGETELEV = 2;
c_Constants.LOC_STANDING = 3;
c_Constants.max_number_objects = 50;


% %% 2a) Load single sensor data
% 
% load('C:\Measurements\2018-10-31_10-04-05_rng_entrance_radar_entrance_14\rng_entrance_radar_entrance_14_2018-10-31_10-04-05_Radar_Locations_LRR5_Corner_Front_Left.mat')
% 
% g_datSWX = dat;
% clear dat
% 
% measurements = load_swx_dat(g_datSWX);


%% 2b) Alternatively load fusion data

% primary datasets directory
dir = '';
if ispc
    dir = '/Users/wam2rng/datasets/';
elseif isunix
    dir = '/home/wam2rng/datasets/';
else
    printf('Could not determine operating system type. Stopping');
    return;
end

% Ring street
%base_name = '2019-08-27-10-50-05';
base_name = 'shuttle_steers_towards_object_merged_';

% Which sensors to load
sensors = ["LRR5_Corner_Front_Left",...
           "LRR5_Corner_Front_Right",...
           "LRR5_Corner_Rear_Left",...
           "LRR5_Corner_Rear_Right"];

measurements_merged = [];
for sensor = sensors
    % make the location string
    load_string = strcat(dir,base_name,sensor,'.mat');
    data = load(load_string);
    measurement = load_swx_dat(data.dat, c_Config, c_Constants);
    measurements_merged = [measurements_merged measurement];
    g_datSWX = data.dat;
end

measurements = sort_array_of_struct(measurements_merged, 't');

for i=1:length(measurements)
    assert( length( measurements(i).asso ) ==512);
end

%% 3) Run object tracker

clc
g_datSWX = radar_tracker_wrapper(measurements, g_datSWX, c_Constants);
dat = g_datSWX;

