function [obj] = mgmt_init_track(obj, measurements, dt) %#codegen
% Track initialization (spawning) module

new_track_init_maxsize = 100;

% store all unassigned measurements 
[new_track_candidates, num_new_tracks] = trackletManagementInit ( measurements, measurements.vx_ego, dt, new_track_init_maxsize );

% are there any new candidates?
if num_new_tracks > 0
    % range/angle/velocity regression or track
    new_track_candidates = new_track_regression(new_track_candidates, num_new_tracks);
    % fuse new candidates, if overlapping
    new_tracks_valid = merge_tracks_init(new_track_candidates, num_new_tracks, new_track_init_maxsize);
    % add new candidates to object list
    obj = set_new_track_properties(obj, new_track_candidates, num_new_tracks, new_tracks_valid,...
        measurements.vx_ego, measurements.t); 
end

end


function [new_track_candidates, num_new_tracks] = trackletManagementInit ( measurements, vx_ego, dt, new_track_init_maxsize )

% defs
tracklet_length = 5;

persistent locations_store
if isempty(locations_store)
    location_mem = struct('r',zeros(tracklet_length,1),'v',zeros(tracklet_length,1),'azimuth_global',zeros(tracklet_length,1),'num_locs_observed',0,'last_update_age',inf,'dt',zeros(tracklet_length,1));
    locations_store = repmat(location_mem, 1000, 1); % list of 1000 location candidates
end

% init output
new_track_candidate = struct('r', zeros(tracklet_length,1), 'v', zeros(tracklet_length,1), 'azimuth_global', zeros(tracklet_length,1), ...
    'dt', zeros(tracklet_length,1), ...
    'r0', 0, 'v0', 0, 'alpha0', 0, 'psi0', 0,...
    'var_r0', inf, 'var_v0', inf, 'var_alpha0', inf);
new_track_candidates = repmat(new_track_candidate,new_track_init_maxsize,1);
num_new_tracks = 0;

% find all non-associated measurements
non_associated_meas_inds = (measurements.asso == 0) & (measurements.meas == 1) & (measurements.potDoubleRefl == 0);
non_associated_meas_inds((measurements.dr < 20) & (measurements.dBRcs < -10)) = false;
non_associated_meas_r = measurements.dr(non_associated_meas_inds);
non_associated_meas_v = measurements.vr(non_associated_meas_inds);
non_associated_meas_ang = measurements.alpha(non_associated_meas_inds);

non_associated_meas_dx = non_associated_meas_r .* cos(non_associated_meas_ang) + measurements.dx_sens_offset;
non_associated_meas_dy = non_associated_meas_r .* sin(non_associated_meas_ang) + measurements.dy_sens_offset;
non_associated_meas_r_global = sqrt(non_associated_meas_dx.^2 + non_associated_meas_dy.^2);

non_associated_location = struct('r',0,'v',0,'azimuth_global',0,'range_gate',0,'vel_gate',0,'ang_gate',0);
non_associated_locations_list = repmat(non_associated_location, sum(non_associated_meas_inds),1);
for non_asso_ind = 1:numel(non_associated_locations_list)
    non_associated_locations_list(non_asso_ind).r = non_associated_meas_r_global(non_asso_ind);
    non_associated_locations_list(non_asso_ind).v = non_associated_meas_v(non_asso_ind);
    non_associated_locations_list(non_asso_ind).azimuth_global = non_associated_meas_ang(non_asso_ind);
    [non_associated_locations_list(non_asso_ind).range_gate, ...
        non_associated_locations_list(non_asso_ind).vel_gate, ...
        non_associated_locations_list(non_asso_ind).ang_gate] = getGateSize(non_associated_meas_r_global(non_asso_ind));
end

% associate measurements to locations_store
non_associated_observed = false(size(non_associated_locations_list));
for loc_store_ind = 1:length(locations_store)
    if locations_store(loc_store_ind).last_update_age == inf
        continue;
    end
    
    curr_alpha = locations_store(loc_store_ind).azimuth_global(locations_store(loc_store_ind).num_locs_observed);
    curr_vel = locations_store(loc_store_ind).v(locations_store(loc_store_ind).num_locs_observed); % ego-motion compensated radial velocity
    locations_store(loc_store_ind).last_update_age = locations_store(loc_store_ind).last_update_age + dt; 
    curr_range = locations_store(loc_store_ind).r(locations_store(loc_store_ind).num_locs_observed) + ...
        curr_vel *locations_store(loc_store_ind).last_update_age; % update 
    
    if locations_store(loc_store_ind).last_update_age > 2 % 2s unobserved
        % delete this tracklet
        locations_store(loc_store_ind).r = zeros(tracklet_length,1);
        locations_store(loc_store_ind).v = zeros(tracklet_length,1);
        locations_store(loc_store_ind).azimuth_global = zeros(tracklet_length,1);
        locations_store(loc_store_ind).num_locs_observed = 0;
        locations_store(loc_store_ind).last_update_age = inf;
        locations_store(loc_store_ind).dt = zeros(tracklet_length,1);
        continue
    end
    
    for meas_ind = 1:length(non_associated_locations_list)
        if locations_store(loc_store_ind).last_update_age ~= inf
            non_associated_meas_r = non_associated_locations_list(meas_ind).r;
            non_associated_meas_v = non_associated_locations_list(meas_ind).v  - vx_ego * (-cos(non_associated_locations_list(meas_ind).azimuth_global));
            non_associated_meas_alpha = wrapToPi_c(non_associated_locations_list(meas_ind).azimuth_global);
            if (abs(non_associated_meas_r - curr_range) < non_associated_locations_list(meas_ind).range_gate) ...
                && (abs(non_associated_meas_v - curr_vel) < non_associated_locations_list(meas_ind).vel_gate) ...
                && (abs(wrapToPi_c(non_associated_meas_alpha - curr_alpha)) < non_associated_locations_list(meas_ind).ang_gate)
                % update this combination!
                locations_store(loc_store_ind).num_locs_observed = locations_store(loc_store_ind).num_locs_observed + 1;
                locations_store(loc_store_ind).dt(locations_store(loc_store_ind).num_locs_observed) = locations_store(loc_store_ind).last_update_age;
                locations_store(loc_store_ind).last_update_age = 0;
                non_associated_observed(meas_ind) = true; % mark as associated

                locations_store(loc_store_ind).r(locations_store(loc_store_ind).num_locs_observed) = non_associated_meas_r;
                locations_store(loc_store_ind).v(locations_store(loc_store_ind).num_locs_observed) = non_associated_meas_v;
                locations_store(loc_store_ind).azimuth_global(locations_store(loc_store_ind).num_locs_observed) = non_associated_meas_alpha;

                % enough consistent locations observed?
                if locations_store(loc_store_ind).num_locs_observed == tracklet_length
                    % add to output
                    new_track_candidate = struct('r', locations_store(loc_store_ind).r, ...
                                                 'v', locations_store(loc_store_ind).v, ...
                                                 'azimuth_global', locations_store(loc_store_ind).azimuth_global, ...
                                                 'dt', locations_store(loc_store_ind).dt,...
                                                 'r0', 0, 'v0', 0, 'alpha0', 0, 'psi0', 0,...
                                                 'var_r0', inf, 'var_v0', inf, 'var_alpha0', inf);
                    num_new_tracks = num_new_tracks + 1;
                    new_track_candidates(num_new_tracks) = new_track_candidate;

                    % delete from locations_store
                    locations_store(loc_store_ind).r = zeros(tracklet_length,1);
                    locations_store(loc_store_ind).v = zeros(tracklet_length,1);
                    locations_store(loc_store_ind).azimuth_global = zeros(tracklet_length,1);
                    locations_store(loc_store_ind).num_locs_observed = 0;
                    locations_store(loc_store_ind).last_update_age = inf;
                    locations_store(loc_store_ind).dt = zeros(tracklet_length,1);
                    if num_new_tracks == new_track_init_maxsize
                        return
                    end
                    continue
                end
            end
        end
    end
end

% create new entries for non-associated measurements 
new_unobserved_locations_list = non_associated_locations_list(not(non_associated_observed));
if isempty(new_unobserved_locations_list)
    return
end
meas_ind = 1;
for loc_store_ind = 1:length(locations_store)
    if locations_store(loc_store_ind).num_locs_observed == 0
        locations_store(loc_store_ind).r(1) = new_unobserved_locations_list(meas_ind).r;
        locations_store(loc_store_ind).v(1) = new_unobserved_locations_list(meas_ind).v - vx_ego * (-cos(new_unobserved_locations_list(meas_ind).azimuth_global));
        locations_store(loc_store_ind).azimuth_global(1) = wrapToPi_c(new_unobserved_locations_list(meas_ind).azimuth_global);
        locations_store(loc_store_ind).num_locs_observed = 1;
        locations_store(loc_store_ind).last_update_age = 0;
        meas_ind = meas_ind + 1;
        if meas_ind > length(new_unobserved_locations_list)
            break
        end
    end
end 
end


function [range_gate_size, vel_gate_size, angle_gate_size] = getGateSize ( range ) 

range_gate_size = max(2, range/10);
angle_gate_size = max(2, range/10) /180*pi;
vel_gate_size = max(3, range/20);

end


function new_tracks_valid = merge_tracks_init( new_track_candidates, num_new_tracks, new_track_init_maxsize )

% defs 
range_thresh = 3; % m
vel_thresh = 2; % m/s
ang_thresh = 3 /180*pi; % 3 degree

% merge
assert(new_track_init_maxsize==100)
track_match = false(100,100); % fixed size for coder not to use problematic memset
for idx1 = 1:num_new_tracks
    for idx2=(idx1+1):num_new_tracks
        if (abs(new_track_candidates(idx1).r0 - new_track_candidates(idx2).r0)) < range_thresh ...
            && (abs(new_track_candidates(idx1).v0 - new_track_candidates(idx2).v0)) < vel_thresh ...
            && (abs(new_track_candidates(idx1).alpha0 - new_track_candidates(idx2).alpha0.')) < ang_thresh
        
            track_match(idx1,idx2) = true; 
        end
    end
end

new_tracks_valid = false(length(new_track_candidates),1);
new_tracks_merge = any(track_match);
new_tracks_valid(1:num_new_tracks) = not(new_tracks_merge(1:num_new_tracks));

end


function new_track_candidates = new_track_regression( new_track_candidates, num_new_tracks )

for track_ind = 1:num_new_tracks
    ranges = new_track_candidates(track_ind).r(end:-1:1);
    velocities = new_track_candidates(track_ind).v(end:-1:1);
    angles = new_track_candidates(track_ind).azimuth_global(end:-1:1);
    meas_age = cumsum([0;new_track_candidates(track_ind).dt(end:-1:2)]);
    
    dx = ranges .* cos(angles); % dx/dy only to calculate heading psi0
    dy = ranges .* sin(angles); 
    delta_dx = dx(end:-1:2) - dx(end-1:-1:1);
    delta_dy = dy(end:-1:2) - dy(end-1:-1:1);
    psi = atan2(delta_dy, delta_dx);
    
    weight = exp(-new_track_candidates(track_ind).dt/0.25); % weights for LS fit; older measurements receive less weight
    
    new_track_candidates(track_ind).alpha0 = angles.' * weight / (sum(weight)); % weighted sample mean
    delta_angles = wrapToPi_c(angles - new_track_candidates(track_ind).alpha0);
    new_track_candidates(track_ind).var_alpha0 = delta_angles.' * diag(weight) * delta_angles /(sum(weight)); % weighted sample variance
    
    new_track_candidates(track_ind).v0 = velocities.' * weight / (sum(weight)); % weighted sample mean
    delta_v0 = velocities - new_track_candidates(track_ind).v0;
    new_track_candidates(track_ind).var_v0 = delta_v0 .' * diag(weight) * delta_v0 /(sum(weight)); % weighted sample variance
    
    ranges_vcomp = ranges + meas_age * new_track_candidates(track_ind).v0;
    new_track_candidates(track_ind).r0 = ranges_vcomp.' * weight / (sum(weight)); % weighted sample mean
    delta_r0 = ranges_vcomp - new_track_candidates(track_ind).r0;
    new_track_candidates(track_ind).var_r0 = delta_r0 .' * diag(weight) * delta_r0 /(sum(weight)); % weighted sample variance
    
    new_track_candidates(track_ind).psi0 = psi.' * weight(1:end-1) / (sum(weight(1:end-1))); 
end

end

function obj = set_new_track_properties(obj, new_track_candidates, num_new_tracks, new_tracks_valid, vx_ego, current_time)

for track_ind = 1:num_new_tracks
    if not(new_tracks_valid(track_ind))
        continue
    end
    % search for free object
    new_obj_idx = 0;
    for obj_idx = 1:length(obj)
        if obj(obj_idx).valid == false
            new_obj_idx = obj_idx;
            break;
        end
    end
    % if object list full and we have a moving object, then delete most distant stationary target
    if new_obj_idx == 0
        delta_v_standing = new_track_candidates(track_ind).v0;
        if abs(delta_v_standing) > 1
            dr2_stat_max = 0;
            for obj_idx = 1:length(obj)
                if (obj(obj_idx).valid == true) && (obj(obj_idx).standing > 0.95)
                    dr2_stat = obj(obj_idx).x(1)^2 + obj(obj_idx).x(4)^2;
                    if dr2_stat > dr2_stat_max
                        dr2_stat_max = dr2_stat;
                        new_obj_idx = obj_idx;
                    end
                end
            end
        end
    end
    
    if new_obj_idx > 0
        % empty object overwrite (to be on the save side)
        obj(new_obj_idx) = get_clean_obj();
        
        dr = new_track_candidates(track_ind).r0;
        vr = new_track_candidates(track_ind).v0;
        alpha = new_track_candidates(track_ind).alpha0;
        
        delta_v_standing = vr;
        if abs(delta_v_standing) < 0.5
            continue;  % ignore stationary objects
        end
        % heuristic: velocity threshold for 0.5 m/s, then exp behavior
        % (P=0.7 at 1m/s for vx_ego=0, P=0.6 at 1m/s for vx_ego=2.7 (Shuttle speed)
        P_moving = 1-exp(-0.5*max(abs(delta_v_standing)-0.5,0)/(0.2 + 0.02*abs(vx_ego)));
        
        dx = dr*cos(alpha); % r is already in global CoSy 
        dy = dr*sin(alpha);

        % Initialize in our direction
        vx = delta_v_standing*cos(alpha);
        vy = delta_v_standing*sin(alpha);

        % object orientation
        psi = atan2(vy,vx);
        obj(new_obj_idx).psi = psi;

        % state vector
        obj(new_obj_idx).x = [dx; vx; 0; dy; vy; 0];

        % transform measurement covariances:  y = [dr vr alpha]'
        var_dr = new_track_candidates(track_ind).var_r0 + 0.5^2;
        var_vr = new_track_candidates(track_ind).var_v0 + 1^2;
        var_vt = 1000;  % high uncertainty for direction!
        var_alpha = new_track_candidates(track_ind).var_alpha0 + (0.25/180*pi)^2;

        R = [var_dr 0       0       0; 
             0      var_vr  0       0;
             0      0       var_vt  0;
             0      0       0       var_alpha];

        dr = sqrt(dx^2 + dy^2);
        vr = sqrt(vx^2 + vy^2);
        vt = 0;  %no tangential velocity
        gs = [cos(alpha)   0           0                (-1)*dr*sin(alpha);
              0            cos(alpha)  (-1)*sin(alpha)  (-1)*(vr*sin(alpha) + vt*cos(alpha));
              sin(alpha)   0           0                dr*cos(alpha);
              0            sin(alpha)  cos(alpha)       (vr*cos(alpha) - vt*sin(alpha))];

        % transform covariance
        Rs = gs*R*gs';

        % consider object extent
        Rs(1,1) = Rs(1,1) + 0.3; %1.0;
        Rs(3,3) = Rs(3,3) + 0.3; %1.0;

        R_final            = [Rs(1,1)   Rs(1,2)     0         Rs(1,3)   Rs(1,4)     0;
                              Rs(2,1)   Rs(2,2)     0         Rs(2,3)   Rs(2,4)     0;
                              0         0           1000      0         0           0;
                              Rs(3,1)   Rs(3,2)     0         Rs(3,3)   Rs(3,4)     0;
                              Rs(4,1)   Rs(4,2)     0         Rs(4,3)   Rs(4,4)     0;
                              0         0           0         0           0         1000];

        obj(new_obj_idx).P = R_final;
        obj(new_obj_idx).valid = true;
        obj(new_obj_idx).meas  = true;
        obj(new_obj_idx).hist  = false;
        obj(new_obj_idx).age  = 0;
        obj(new_obj_idx).t_abs = current_time;
        obj(new_obj_idx).t_lastmeas = current_time;
        obj(new_obj_idx).standing  = (1 - P_moving);
        obj(new_obj_idx).moving  = P_moving;
        obj(new_obj_idx).pexist  = 0.2;
        obj(new_obj_idx).length = 1; 
        obj(new_obj_idx).width = 1; 
        obj(new_obj_idx).psiDt = 0;
        obj(new_obj_idx).P_psi = [(pi)^2 0; 0 1];
        obj(new_obj_idx).P_obj_type = [0.5 0.5];

        % IMM-Models initialization
        % #1 constant turn rate
        P_turn = [R_final(1,1)  R_final(1,4) 0       0      0           0;
                  R_final(4,1)  R_final(4,4) 0       0      0           0;
                  0             0           (10/180*pi)^2   0      0           0;
                  0             0           0        1      0           0;
                  0             0           0        0      var_vt      0;
                  0             0           0        0      0           0];
        obj(new_obj_idx).IMM(1).x = [dx; dy; psi; 0; vr; 0];
        obj(new_obj_idx).IMM(1).P = P_turn;
        obj(new_obj_idx).IMM(1).mu = 1/3;
        % #2 crossing high-dyn
        obj(new_obj_idx).IMM(2).x = obj(new_obj_idx).x;
        obj(new_obj_idx).IMM(2).P = obj(new_obj_idx).P;
        obj(new_obj_idx).IMM(2).mu = 1/3;
        % #3 crossing low-dyn
        obj(new_obj_idx).IMM(3).x = obj(new_obj_idx).x;
        obj(new_obj_idx).IMM(3).P = obj(new_obj_idx).P;
        obj(new_obj_idx).IMM(3).mu = 1/3;

        % combine IMM results
        obj(new_obj_idx) = imm_output(obj(new_obj_idx), vx_ego);

        % init local gridmap
        obj(new_obj_idx).grid.LR = 0.1*ones(size(obj(new_obj_idx).grid.LR));      
        obj(new_obj_idx).grid.LR(ceil(obj(new_obj_idx).grid.x_len/2), ceil(obj(new_obj_idx).grid.y_len/2)) = 1;

        % RCS initialize
        obj(new_obj_idx).RCS_filt = -128;
    end
end

end






