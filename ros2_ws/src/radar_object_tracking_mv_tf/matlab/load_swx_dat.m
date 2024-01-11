% Laden der Messwerte aus der datSWX-Struktur (alle Winkelmesswerte)
%
% \param number_cycles_for_import import max. of this number of cycles from the SWX data.
function [measurements] = load_swx_dat(datSWX, c_Config, c_Constants, number_cycles_for_import)

    % Check amount of data to import
    if nargin < 4
        number_cycles_for_import = c_Config.number_cycles_for_import;
    end
    number_cycles_for_import = min(number_cycles_for_import, length(datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.tAbsMeasTimeStamp));

    % Init
    measurements = struct;

    % Todo Joachim B�rger 13.11.2018 ge�ndert, keine Ahnung weshalb das -1, aber die Liste muss
    % genausolang sein wie die Locationliste, damit auch eine gleichlange Objektliste berechnet wird.
    % Alles andere kann SWX (derzeit) nicht plotten
    % for cycle = 1:length(datSWX.bpc_general.CounterTC)-1
    for cycle = 1:number_cycles_for_import

        ShowProgress(number_cycles_for_import, cycle, 'load_swx_dat');

        %% init of data structure
        measurements(cycle).asso          = zeros(1, c_Config.number_locations);     % Assoziations-Flag
        measurements(cycle).preasso       = zeros(1, c_Config.number_locations);     % Assoziations-Flag
        measurements(cycle).potGhost      = zeros(1, c_Config.number_locations);
        measurements(cycle).potDoubleRefl = zeros(1, c_Config.number_locations);

        measurements(cycle).meas  = bitand(datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.Item.measStatus(1:c_Config.number_locations, cycle)', 2^c_Constants.LOC_MEASURED);

        % Zeilenvektoren
        measurements(cycle).dr     = datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.Item.d(1:c_Config.number_locations, cycle)';
        measurements(cycle).drVar  = f16_to_f32( datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.Item.dVar.m_value(1:c_Config.number_locations, cycle) )';
        measurements(cycle).vr     = datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.Item.v(1:c_Config.number_locations, cycle)';
        measurements(cycle).vrVar  = f16_to_f32( datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.Item.vVar.m_value(1:c_Config.number_locations, cycle) )';


        % Todo Joachim B�rger 26.10.2018 Find better default or even better adaptation to Gen5
        measurements(cycle).PDH1 = repmat(0.95, 1, c_Config.number_locations);
        measurements(cycle).PDH0 = repmat(0.04, 1, c_Config.number_locations);        

        measurements(cycle).dBRcs = f16_to_f32( datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.Item.RCS.m_value(1:c_Config.number_locations, cycle) )';
%         measurements(cycle).dBRcs(2:6, :) = 0;
        

        %
%         measurements(cycle).t = datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.tRadarECUTimeStamp(cycle); % datSWX.lrr_general.tAbsMeasTimeStamp(cycle)/(2^16);
        measurements(cycle).t = datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.tROSTimeStampRelative(cycle); % datSWX.lrr_general.tAbsMeasTimeStamp(cycle)/(2^16);
%         measurements(cycle).t = datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.tAbsMeasTimeStamp(cycle); % datSWX.lrr_general.tAbsMeasTimeStamp(cycle)/(2^16);
        
        % Use absolute ROS time for fusion
%         measurements(cycle).tROSTimeStampAbsolute = datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.tROSTimeStampAbsolute(cycle); % datSWX.lrr_general.tAbsMeasTimeStamp(cycle)/(2^16);

        
        %% Ego motion data
        if isfield(datSWX, 'ego_state_out')
            measurements(cycle).vx_ego = datSWX.ego_state_out.velocity.x(1,cycle); % datSWX.ohl_General.vSync(cycle)/256;
            measurements(cycle).ax_ego = datSWX.ego_state_out.aT(1,cycle); %datSWX.ohl_General.aSync(cycle)/2048;
            measurements(cycle).psiDt_ego = datSWX.ego_state_out.angular_velocity.z(1,cycle); % datSWX.ohl_General.psiDtOptSync(cycle)/16384;
            measurements(cycle).kapCurvTraj = datSWX.ego_state_out.dcurvature(1,cycle); % datSWX.evi_MovementData.kapCurvTraj(cycle)/262144;
            measurements(cycle).beta_ego = 0; % datSWX.evi_MovementData.alpSideSlipAngle(cycle)/65536;

        else
            measurements(cycle).vx_ego = 0; % datSWX.ohl_General.vSync(cycle)/256;
            measurements(cycle).ax_ego = 0; %datSWX.ohl_General.aSync(cycle)/2048;
            measurements(cycle).psiDt_ego = 0; % datSWX.ohl_General.psiDtOptSync(cycle)/16384;
            measurements(cycle).kapCurvTraj = 0; % datSWX.evi_MovementData.kapCurvTraj(cycle)/262144;
            measurements(cycle).beta_ego = 0; % datSWX.evi_MovementData.alpSideSlipAngle(cycle)/65536;

        end
        
        %% Radar Mounting Position

        measurements(cycle).dx_sens_offset = datSWX.g_PerSppRLocGen5Runnable_m_radarSensorPropertiesPort_out.m_currentMounting.m_vectorCovariancePair.m_muVector(1, cycle);
        measurements(cycle).dy_sens_offset = datSWX.g_PerSppRLocGen5Runnable_m_radarSensorPropertiesPort_out.m_currentMounting.m_vectorCovariancePair.m_muVector(2, cycle);
        measurements(cycle).ang_sens_offset = datSWX.g_PerSppRLocGen5Runnable_m_radarSensorPropertiesPort_out.m_currentMounting.m_vectorCovariancePair.m_muVector(4, cycle);


        %% FOV
        
        measurements(cycle).fov_range =         f16_to_f32( datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_SensState.FieldOfView_st.dMaxThetaView.m_value(:,cycle)).';
        measurements(cycle).fov_angle = deg2rad(f16_to_f32( datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_SensState.FieldOfView_st.thetaViewAry.m_value(:,cycle))).';

%         fov_angle_bound = mean([fov_angle(2:end), fov_angle(1:end-1)], 2);
%         fov_angle_bound = [-90; fov_angle_bound; 90];
%         
%         fov_angle_lower = fov_angle_bound(1:end-1);
%         fov_angle_higher = fov_angle_bound(2:end);
%         
%         measurements(cycle).angFOV_dBorder = fov_range.';
%         measurements(cycle).angFOV_alpUpper = deg2rad(fov_angle_higher).';
%         measurements(cycle).angFOV_alpLower = deg2rad(fov_angle_lower).';
% 
%         % Workaround for incorrect ego-velocity at low speeds.
%         %************************************************************
%         %check this again. Is this correct
%         if abs(measurements(cycle).vx_ego) < c_Config.vx_ego_deadzone
%             measurements(cycle).vx_ego = 0;
%             measurements(cycle).ax_ego = 0;
%         end

        
        % Theta ist in Deg
        theta = f16_to_f32( datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.Item.theta.m_value(1:c_Config.number_locations, cycle) );
        
        % Consider corner sensors mounted top-down
        if datSWX.g_PerSppRLocGen5Runnable_debugvariables.m_sppRLocGen5Param.m_isOrientationFlippedInRollangle
            measurements(cycle).alpha(1, :) = -deg2rad(theta);
            measurements(cycle).fov_angle = -measurements(cycle).fov_angle(end:-1:1); % also FOV flipped
            measurements(cycle).fov_range = measurements(cycle).fov_range(end:-1:1); % also FOV flipped
        else
            measurements(cycle).alpha(1, :) = deg2rad(theta);
        end        
        measurements(cycle).alpha = measurements(cycle).alpha + measurements(cycle).ang_sens_offset;
       
        
        % ThetaVar ist die Variance in Deg^2
        thetaVar = f16_to_f32( datSWX.g_PerSppRLocGen5Runnable_m_ensRLocationsPort_in.m_LocationList.Item.thetaVar.m_value(1:c_Config.number_locations, cycle) );
        measurements(cycle).alpVar(1, :) = deg2rad(deg2rad(thetaVar));
    end
end




