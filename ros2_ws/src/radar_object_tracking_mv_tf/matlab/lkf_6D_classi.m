function [ obj_list ] = lkf_6D_classi( obj_list, svm_model, PROPERTIES ) %#codegen
% Klassifikation des Objekttyps
% obj_type(1) : car/truck
% obj_type(2) : bicycle

for obj_nr = 1:PROPERTIES.NUM_OBJ
    if obj_list(obj_nr).valid == true
        
        if (obj_list(obj_nr).meas == true) && (obj_list(obj_nr).moving > 0.9) % Nur bei Gemessen-Flag ist neue Information vorhanden

            
                obj_dr = sqrt(obj_list(obj_nr).x(1)^2 + obj_list(obj_nr).x(4)^2);
                obj_vabs = sqrt(obj_list(obj_nr).x(2)^2 + obj_list(obj_nr).x(5)^2);

                % Likelihood der Klassen bestimmen  
                LR = obj_list(obj_nr).P_obj_type(1)/obj_list(obj_nr).P_obj_type(2);

            %     % RCS: Differenz zur Klassengrenze ermitteln
            %     % g1 = 12/30*x - 7  [0...30]
            %     % g2 = 3/50*x + 3.2  [30...80]
            %     if obj_dr < 30
            %         diff_RCS = obj_list(obj_nr).RCS_filt - (12/30*obj_dr - 7); 
            %     else
            %         diff_RCS = obj_list(obj_nr).RCS_filt - (3/50*obj_dr + 3.2);
            %     end
            %     if diff_RCS > 0
            %         LR = LR * 2;
            %     else
            %         LR = LR * 0.5;
            %     end
            %     
            %     
            %     % L�nge: Differenz zur Klassengrenze ermitteln
            %     % g1 = 0.7 [0 ...17]
            %     % g2 = -0.7/23*x + 1.22 [17...40]
            %     % g3 = 0.1 [40...]
            %     if obj_dr < 17
            %         diff_len = obj_list(obj_nr).length_meas - 0.7; 
            %     elseif obj_dr < 40
            %         diff_len = obj_list(obj_nr).length_meas - (-0.7/23*obj_dr + 1.22);
            %     else
            %         diff_len = obj_list(obj_nr).length_meas - 0.1;
            %     end
            %     if diff_len > 0
            %         LR = LR * 2;
            %     else
            %         LR = LR * 0.5;
            %     end

                % Feature-Veaktor
                obj_dr = sqrt((obj_list(obj_nr).x(1)-3.8)^2 + obj_list(obj_nr).x(4)^2);
                p = [obj_dr; obj_list(obj_nr).RCS_filt; obj_list(obj_nr).length_meas]; %Aktueller Feature-Vektor

                
                % Features auf -1..1 skalieren (Ohne Skalierung)
                % p = scale_feature(p);

            %     % Methode 1: Abstand zur Trennebene �ber Normalenvektor         
            %     n = [35; -350; -2000]; % Normalenvektor der Trennebene
            %     n0 = n/norm(n);
            %     d0 = -700/norm(n);
            %     d = dot(p,n0) - d0;  % Abstand zur Trennebene
            %     LRp = 4; 
            %     if d < 0
            %         LR = LR * LRp; % PKW
            %     else
            %         LR = LR / LRp; % Fahrrad
            %     end  

            % %     % Methode 2: Trenn-Ebene im 3D-Feature-Raum
            % %     len_trenn = 35/2000*obj_dr - 35/200*obj_list(obj_nr).RCS_filt + 7/20;
            % %     if obj_list(obj_nr).length_meas > len_trenn
            % %         LR = LR * 4; % PKW
            % %     else
            % %         LR = LR * 0.25; % Fahrrad
            % %     end

                % Methode 3: SVM 
                [pred_class, accuracy, prob_estimates] = svmpredict(1,p',svm_model,'-q -b 1'); % -b 1');  % Bessere Performance ohne -b 1              
                
                LRp = (1 + prob_estimates(1))/(1 + prob_estimates(2));
                %obj_list(obj_nr).P_bike_cur = prob_estimates(2);
                LR = LR * LRp;
%                 if pred_class == 1
%                     LR = LR * LRp; % PKW
%                     % LR = 10;
%                 else
%                     LR = LR / LRp; % Fahrrad
%                     % LR = 1/10;
%                 end 
               % prob_estimates


                % Geschwindigkeit: Objekte �ber 50km/h sind vermutlich keine
                % Fahrradfahrer
                if obj_vabs > 35/3.6
                    LR = LR * (1 + (obj_vabs - 35/3.6)*0.25); 
                    % Bei 35km/h Faktor 1, Bei 50km/h Faktor 2, bei 100 Faktor 5.5
                end


                % Klassenwahrscheinlichkeiten bestimmen
                obj_list(obj_nr).P_obj_type(1) = LR/(1 + LR)*0.999;
                obj_list(obj_nr).P_obj_type(2) = 1 - obj_list(obj_nr).P_obj_type(1);
            end
    

    end

end

end














