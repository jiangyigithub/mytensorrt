function obj = locgrid_recenter(obj)
%LOCGRID_RECENTER recenters the objects local gridmap, such that the center
%of gravity of the likelihood is near zero. 

anz_IMM_models = 3;
likelihood_floor = 0.1; % minimum value of LR grid

likelihood = obj.grid.LR ./ (1 + obj.grid.LR);  % convert odds to prob
likelihood = (likelihood - likelihood_floor) / (1-likelihood_floor);

x_likelihood = sum(likelihood,2);
y_likelihood = sum(likelihood,1);
x_mean_likelihood = sum(x_likelihood .* obj.grid.x_cells.')/sum(x_likelihood);
y_mean_likelihood = sum(y_likelihood .* obj.grid.y_cells)/sum(y_likelihood);


x_shift_bins = -round(x_mean_likelihood / obj.grid.step);
y_shift_bins = -round(y_mean_likelihood / obj.grid.step);

if (x_shift_bins~=0)||(y_shift_bins~=0)
    % shift grid
    
    obj.grid.LR = shift_grid(obj.grid.LR, x_shift_bins, 1, likelihood_floor);
    obj.grid.LR = shift_grid(obj.grid.LR, y_shift_bins, 2, likelihood_floor);
    
    % correct object px, py for shift
    
    x_shift_local = -x_shift_bins * obj.grid.step;
    y_shift_local = -y_shift_bins * obj.grid.step;
    x_shift_global =  cos(-obj.psi) * x_shift_local + sin(-obj.psi) * y_shift_local;
    y_shift_global = -sin(-obj.psi) * x_shift_local + cos(-obj.psi) * y_shift_local;
    
    obj.x(1) = obj.x(1) + x_shift_global;
    obj.x(4) = obj.x(4) + y_shift_global;
    obj.IMM(1).x(1) = obj.IMM(1).x(1) + x_shift_global;
    obj.IMM(1).x(2) = obj.IMM(1).x(2) + y_shift_global;
    for i=2:anz_IMM_models
        obj.IMM(i).x(1) = obj.IMM(i).x(1) + x_shift_global;
        obj.IMM(i).x(4) = obj.IMM(i).x(4) + y_shift_global;
    end
end


end

function grid_out = shift_grid(grid_in, shift_num_bins, shift_dim, new_cells_fill_value)

assert((shift_dim ==1) || (shift_dim==2));

if(shift_num_bins == 0)
    grid_out = grid_in;
    return
end

grid_out = new_cells_fill_value * ones(size(grid_in));

if shift_dim == 1
    if shift_num_bins > 0
        grid_out(1+shift_num_bins:end, :) = grid_in(1:(end-shift_num_bins), :);
    else
        grid_out(1:(end+shift_num_bins), :) = grid_in((1-shift_num_bins):end, :);
    end
elseif shift_dim == 2
    if shift_num_bins > 0
        grid_out(:, 1+shift_num_bins:end) = grid_in(:, 1:(end-shift_num_bins));
    else
        grid_out(:, 1:(end+shift_num_bins)) = grid_in(:, (1-shift_num_bins):end);
    end
end

end

