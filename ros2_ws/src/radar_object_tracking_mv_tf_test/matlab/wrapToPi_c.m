function angle = wrapToPi_c(angle)
% substitute for matlab's wrapToPi, which is not supported for code
% generation


for elem_ind = 1:length(angle)
    while angle(elem_ind) < -pi
        angle(elem_ind) = angle(elem_ind) + 2*pi;
    end

    while angle(elem_ind) > pi
        angle(elem_ind) = angle(elem_ind) - 2*pi;
    end
end

end