% Append the object data computed in this cycle to the SWX data struct (which contains all data from
% start of computation!)
function g_datSWX = copy_obj2swx_dat(g_datSWX, c_Constants, obj_list, meas_list, cycle)

    % Without this, SWX does not plot
    g_datSWX.g_PerSpdRunnable_m_syncInfoPort_out.m_cycle_index(1, cycle) = cycle;
    g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.tAbsRefTime_ul(1, cycle) = meas_list.t;

    for obj_nr = 1:50

        % Koordinatensystem Matlab dat: Ursprung Vorderkante, vx-relativ, vy-absolut

        % Ziel: O x T
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.dxv_sw(obj_nr, cycle) = (obj_list(obj_nr).x(1) - meas_list.dx_sens_offset) * 2^7;
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.vxv_sw(obj_nr, cycle) = (obj_list(obj_nr).x(2) - meas_list.vx_ego) * 2^8;
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.axv_sw(obj_nr, cycle) = obj_list(obj_nr).x(3) * 2^11;
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.dyv_sw(obj_nr, cycle) = obj_list(obj_nr).x(4) * 2^7;
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.vyv_sw(obj_nr, cycle) = obj_list(obj_nr).x(5) * 2^8;
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.ayv_sw(obj_nr, cycle) = obj_list(obj_nr).x(6) * 2^11;


        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.wExistProb_uw(obj_nr, cycle) = obj_list(obj_nr).pexist * 2^16;
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.pMoving(obj_nr, cycle) = obj_list(obj_nr).moving * 2^7;

        % Todo probably we want to keep the max. here
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.prob1HasBeenObservedMoving_ub(obj_nr, cycle) = 1 * obj_list(obj_nr).moving * 2^7; % obj_list(obj_nr).moving * 2^7;

        % New, initial guess and might need to be improved in order to separate the line plots for different objects
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.HandleOld_ub(obj_nr, cycle) = obj_nr;
        if ~obj_list(obj_nr).valid 
            g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.HandleOld_ub(obj_nr, cycle) = 0;
        end
        
        % Geschickte Vergabe von Handles, damit SWX bei neu entstandenen Objekten die
        % Verbindungslinie unterbricht

        % Neues Objekt entstanden?

        % Im letzten Zyklus gab es bereits ein g�ltiges Objekt VERSCHIEDEN vom diesigen
        bObjectExistedBefore = false;
        if cycle > 1
            bObjectExistedBefore = g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.Handle_ub(obj_nr, cycle-1) > 0;
        end

        % 4 F�lle: Objekt ist ung�ltig...
        if ~obj_list(obj_nr).valid
            Handle = 0;
            
        % Vorher gab es kein Objekt, dann freie Handle-Wahl
        elseif ~bObjectExistedBefore 
            Handle = obj_nr;
            
        % oder vorher gab es dasselbe Objekt, dann Handle beibehalten
        elseif obj_list(obj_nr).hist
            Handle = g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.Handle_ub(obj_nr, cycle-1);
         
        % oder vorher gab es ein anderes Objekt, dann alternatives Handle (+50) nehmen. Durch die
        % MOD-Operation ist gew�hrt, dass die Handles sich im Bereich [1, 100] bleiben
        else
            Handle = g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.Handle_ub(obj_nr, cycle-1) + c_Constants.max_number_objects;
            Handle = mod(Handle-1, 2 * c_Constants.max_number_objects)+1;
        end

        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.Handle_ub(obj_nr, cycle) = Handle;
        

        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.b_un.b_st.Measured_b(obj_nr, cycle) = obj_list(obj_nr).meas;
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.dLength_sw(obj_nr, cycle) = obj_list(obj_nr).length * 2^7;
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.dWidth_sw(obj_nr, cycle) = obj_list(obj_nr).width * 2^7;

         % Todo conversion
        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.yawAngle(obj_nr, cycle) = obj_list(obj_nr).psi * 2^13 * pi / 180;

        g_datSWX.g_PerEimTprRunnable_m_fusObjPort_out.FusObj_st.tAbsRefTime_ul(obj_nr, cycle) = obj_list(obj_nr).t_abs * 2^16;

    end
end


