function [obj] = mgmt_pex(obj,asso_list) %#codegen
% update existance probability

update_smoothing_factor = 0.1; % for smoothing pexist 

if obj.measureable
    measurement_volume = obj.asso.range_interval * obj.asso.vel_interval * obj.asso.ang_interval ...
        / 0.4 / 0.2 / (8*pi/180); % range, Doppler and azimuth separability
    measurement_volume = max(1,measurement_volume);
    range = sqrt(obj.x(1)^2 + obj.x(4)^2);
    num_targets_observed = min(obj.meas_angles, length(asso_list));
    
    % get PDH1, PDH0
    [PDH1, PDH0] = gen5_obj_detection_existence_heuristic(range, obj.P_obj_type(1), obj.P_obj_type(2),measurement_volume);

    % convert probability to log likelihood ratio
    LLR = log(obj.pexist/(1-obj.pexist)); 
    for asso_cnt = 1:num_targets_observed
        [S, d2] = imm_merge_kinematic_fit(asso_list(asso_cnt).S, asso_list(asso_cnt).d2, obj.IMM);

        % likelihood ratio update for kinematic fit
        % blackman/popoli Eq. 6.6
        LR_K = measurement_volume*exp(-d2/2) / ((2*pi)^(length(d2)/2) * sqrt(det(S))); 

        % likelihood ratio update for signal part
        % blackman/popoli Eq. 6.10a
        LR_S = PDH1/PDH0;

        LLR_U = log(LR_K) + log(LR_S);
        
        LLR_U = max(LLR_U, 0.1); % ensure small incremant

        % sanity check
        if ~isfinite(LLR_U)
            continue;
        end

        LLR = LLR + update_smoothing_factor * LLR_U;
    end

    expected_target_area = obj.length * obj.width;
    expected_num_targets = expected_target_area * exp(-range/50); % heuristic 
    if range > 50
        expected_num_targets = max(expected_num_targets, 1);
    elseif range > 20
        expected_num_targets = max(expected_num_targets, 2);
    else
        expected_num_targets = max(expected_num_targets, 3);
    end

    if expected_num_targets > num_targets_observed

        % likelihood update for non-observed
        % blackman/popoli Eq 6.10b
        LR_S_single = (1-PDH1)/(1-PDH0); 

        LLR_S_single = log(LR_S_single);
        
        LLR_S_single = min(LLR_S_single, -0.1); % ensure small decrement

        LLR = LLR + update_smoothing_factor * (expected_num_targets - num_targets_observed) * LLR_S_single;
    end

    % convert log-likelihood ratio to probability
    obj.pexist = exp(LLR)/(1+exp(LLR)); 
    
    % limit pexist
    obj.pexist = min(obj.pexist, 0.995);
end

end

function [PDH1,PDH0] = gen5_obj_detection_existence_heuristic(range, p_car, p_bike, object_volume)
% object and non-detection probability of an object at a certain range. 
% PDH0: Probability, that an object does not exist, although detected (false alarm probability)
% PDH1: Probability, that an object does exist, if detected (detection probability)

%% PDH1 heuristic
% PDH car
expected_range_car = 80; % meter, distance where detection and non-detection are equally likely, gen5 corner
PDH1_car = 0.99 * exp(-range/(expected_range_car)/5); % PDH1 reaches 0.8 at expected_range

% PDH bike
expected_range_bike = 30; % meter, distance where detection and non-detection are equally likely, gen5 corner
PDH1_bike = 0.99 * exp(-range/(expected_range_bike)/5);

% PDH unknown 
expected_range_unknown = 50; % meter, distance where detection and non-detection are equally likely, gen5 corner
PDH1_unknown = 0.99 * exp(-range/(expected_range_unknown)/5); 

if (p_car + p_bike) > 1
    p_norm = p_car + p_bike;
    p_car = p_car / p_norm;
    p_bike = p_bike / p_norm;
    p_other = 0;
else
    p_other = 1 - p_bike - p_car;
end

PDH1 = PDH1_car * p_car + PDH1_bike * p_bike + PDH1_unknown * p_other;

%% PDH0 calculation

% def:
beta_false_alarm = 0.005;

% calc 
PDH0 = beta_false_alarm * object_volume;
PDH0 = min(PDH0, PDH1*0.1);

end