function [obj] = mgmt_del(obj, vx_ego, current_time) %#codegen
% delete irrelevant objects
% merge nearby objects

num_objs = 0;
for obj_idx = 1:length(obj)
    if obj(obj_idx).valid == 1
        num_objs = num_objs + 1;
    end
end

for obj_idx = 1:length(obj)

    if obj(obj_idx).valid == 1
        
        dr_obj  = sqrt(obj(obj_idx).x(1)^2 + obj(obj_idx).x(4)^2);
        obj_ang = atan2(obj(obj_idx).x(4),obj(obj_idx).x(1));
        vr_obj  = cos(obj_ang)*(obj(obj_idx).x(2)-vx_ego) + sin(obj_ang)*obj(obj_idx).x(5);
        
        position_tolerance_max = max(5, dr_obj / 10); % max of 5m and range/10
            
        % delete unlikely objects
        if obj(obj_idx).pexist < 0.025
            obj(obj_idx) = get_clean_obj();
        elseif (obj(obj_idx).pexist < 0.15) && (num_objs > 30)
            obj(obj_idx) = get_clean_obj();
        elseif (obj(obj_idx).pexist < 0.2) && (num_objs > 45)
            obj(obj_idx) = get_clean_obj();
        % delete objects which are not updated (out of FOV)
        elseif abs(obj(obj_idx).t_lastmeas - current_time) > 0.75  % s
            obj(obj_idx) = get_clean_obj();
        % delete objects outside region of interest
        elseif (dr_obj > 200)  % be careful: depends on sensor type!
            obj(obj_idx) = get_clean_obj();
        elseif (obj(obj_idx).P(1,1) > position_tolerance_max^2) || (obj(obj_idx).P(4,4) > position_tolerance_max^2) % object uncertainty for position
            obj(obj_idx) = get_clean_obj();
        else
            % Merge nearby objects
            for obj_idx2 = 1:length(obj)
                if (obj(obj_idx2).valid == 1) && (obj_idx2 ~= obj_idx)                            
                    % Delete / Merge ----------------------------------------------------------------
                    if (obj(obj_idx).moving >= 0.90) && (obj(obj_idx2).moving >= 0.90)
                        moving_objects = 1;
                        dxdy_absGate = 5;
                        vxvy_absGate = 3;
                    else
                        moving_objects = 0;
                        dxdy_absGate = 1;
                        vxvy_absGate = 1;
                    end
                    
                    dr_obj2 = sqrt(obj(obj_idx2).x(1)^2 + obj(obj_idx2).x(4)^2);
                    obj_ang2 = atan2(obj(obj_idx2).x(4),obj(obj_idx2).x(1));
                    vr_obj2  = cos(obj_ang2)*(obj(obj_idx2).x(2)-vx_ego) + sin(obj_ang2)*obj(obj_idx2).x(5);

                    delta_alpha = abs(obj_ang - obj_ang2);
                    if delta_alpha > pi
                        delta_alpha = 2*pi - delta_alpha;
                    end
                    
                    % are objects nearby?
                    if   (   (((obj(obj_idx).x(1)-obj(obj_idx2).x(1))^2 + (obj(obj_idx).x(4)-obj(obj_idx2).x(4))^2) < (dxdy_absGate^2)) ...
                          && (abs(obj(obj_idx).x(2)-obj(obj_idx2).x(2)) < vxvy_absGate) && (abs(obj(obj_idx).x(5)-obj(obj_idx2).x(5)) < vxvy_absGate) ) ...
                       || (   (abs(dr_obj - dr_obj2) < 0.2)  ...     % are objects separable by radar sensor?
                            && (abs(vr_obj - vr_obj2) < 0.1) ...
                            && (delta_alpha < (10/180*pi)) ...
                            && (moving_objects == 1))                     

                        
                            prio1 = 0;
                            prio2 = 0;
                            
                            if (obj(obj_idx).pexist > obj(obj_idx2).pexist)  % higher p_exist has priority
                                prio1 = prio1 + 1;
                            end
                            if (obj(obj_idx2).pexist > obj(obj_idx).pexist)  % higher p_exist has priority
                                prio2 = prio2 + 1;
                            end
                            
                            if (obj(obj_idx).moving > obj(obj_idx2).moving) % moving objects have priority
                                prio1 = prio1 + 1;
                            end
                            if (obj(obj_idx2).moving > obj(obj_idx).moving) % moving objects have priority
                                prio2 = prio2 + 1;
                            end
                            
                            if (obj(obj_idx).age > obj(obj_idx2).age) % old objects have priority
                                prio1 = prio1 + 1;
                            end
                            if (obj(obj_idx2).age > obj(obj_idx).age) % old objects have priority
                                prio2 = prio2 + 1;
                            end

                            % delete objects with lower prio. If prio is equal, the age decides
                            if (prio1 > prio2)
                                obj(obj_idx2) = get_clean_obj();
                            elseif (prio2 > prio1)
                                obj(obj_idx) = get_clean_obj();
                                break;
                            end

                    end
                end
            end
        end

    end
end



