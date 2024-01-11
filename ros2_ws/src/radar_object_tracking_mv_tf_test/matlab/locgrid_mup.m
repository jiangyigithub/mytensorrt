function obj = locgrid_mup(obj, asso, obj_idx, sensor_pos_offset)
    
% parameter definitions
PDH1 = 0.7;
PDH0 = 0.15; 


% load objectproperties
obj_px = obj.x(1);
obj_py = obj.x(4);
obj_orntn = obj.psi;

for meas_ind = 1:length(asso)
    if asso(meas_ind).loc_nr > 0
        % determine indices
        [refl_x, refl_y] = ekf_6D_CA_meas2xy(asso(meas_ind).y(1), asso(meas_ind).y(2), sensor_pos_offset(1), sensor_pos_offset(2));

        refl_x_no_offset = refl_x - obj_px; % both is in global coordinates
        refl_y_no_offset = refl_y - obj_py; 

        refl_x_obj_local =  cos(obj_orntn) * refl_x_no_offset + sin(obj_orntn) * refl_y_no_offset; % in object local cosy
        refl_y_obj_local = -sin(obj_orntn) * refl_x_no_offset + cos(obj_orntn) * refl_y_no_offset;

        LR_update = PDH1/PDH0;

        [~,refl_x_bin] = min(abs(obj.grid.x_cells.' - refl_x_obj_local));
        [~,refl_y_bin] = min(abs(obj.grid.y_cells.' - refl_y_obj_local));

        % update
        obj.grid.LR(refl_x_bin,refl_y_bin) = obj.grid.LR(refl_x_bin,refl_y_bin) * LR_update;
    end
end

% % plot
% likelihood = obj.grid.LR ./ (1 + obj.grid.LR);  % convert odds to prob
% figure(100+obj_idx)
% contourf(obj.grid.x_cells,obj.grid.y_cells,likelihood.',100,'linestyle','none')
% caxis([0,1]);
% xlabel('x in object cosy')
% ylabel('y in object cosy')
% xticks(-5:5)
% yticks(-5:5)
% axis equal
% title(sprintf('r = %f',sqrt(obj_px^2+obj_py^2)))
% colorbar

end