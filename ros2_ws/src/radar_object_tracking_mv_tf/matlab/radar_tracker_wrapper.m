function g_datSWX = radar_tracker_wrapper(measurements, g_datSWX, c_Constants)

% global svm_model;


% ------------------------------------------------------------------------


% Todo Joachim B�rger 26.10.2018: Pfade korrigieren
% load svm_model_V11.mat;
% load('C:\Sandboxes\Kreuzungsassistent\perception\code\Matlab\Classifikation_stuff\svm_model_V11.mat');
%load svm_model_verdeckt.mat;
%load svm_model_MRRevo14.mat;

first_cycle = 1;
last_cycle = length(measurements);
% last_cycle = min(10000,length(measurements));


% Hauptschleife
for cycle = first_cycle:last_cycle
    fprintf('\n Zyklus %i: ',cycle);

    ShowProgress(last_cycle, cycle, 'radar_tracker_wrapper')

    meas_list = measurements(cycle); % b

    if (cycle == first_cycle)
        % init obj list
        obj_list = radar_tracker_init();
        dt = 0.070;
        t_last = measurements(cycle).t;
        cycle_out = first_cycle;
    else
        dt = measurements(cycle).t - t_last;
        t_last = measurements(cycle).t;


        % delayed measurement or radar ECU timestamp reset (after ca. 4100s)
        if abs(dt) > 0.5
            fprintf('Zykluszeit zu lang, Reset auf 0.07! dt = %1.3f\n',dt);
            dt = 0.070;
        end

        if dt <= 0.0
            fprintf('OOSM: dt=%1.3f\n',dt);
        end

        cycle_out = cycle_out + 1;
    end

    % main loop: call filter
    obj_list = radar_tracker(obj_list, meas_list, dt);

    % alternatively call mex-function for speed-up
    % obj_list = radar_tracker_mex('radar_tracker',obj_list, meas_list, dt);

      
    % Objekttyp �ber SVM klassifizieren
%     [ obj_list ] = lkf_6D_classi( obj_list, svm_model );


    % export results in dat structure fr swx debugging
    g_datSWX = copy_obj2swx_dat(g_datSWX, c_Constants, obj_list, meas_list, cycle_out);

end

fprintf('\n');

assignin('base','obj_list',obj_list);
assignin('base','meas_list',meas_list);


