function [feature_vec_scaled] = scale_feature(feature_vec)

    % Minima und Maxima der trainierten Features
    x_min_max = [0.172188   81.4036;
                 -17.4325   33.8382;
                 0          5.19849];
    % Minima und Maxima der skalierten Freatures
    lower = -1;
    upper = 1;
        
    % Leeren Feature-Vektor erzeugen
    feature_vec_scaled = zeros(size(feature_vec));
    
    for idx = 1:length(feature_vec)
  
       value = feature_vec(idx);
       x_min = x_min_max(idx,1);
       x_max = x_min_max(idx,2);
       
       value_scaled = lower + (upper - lower)*(value - x_min)/(x_max - x_min); 
  
       feature_vec_scaled(idx) = value_scaled;
       
    end       

end
