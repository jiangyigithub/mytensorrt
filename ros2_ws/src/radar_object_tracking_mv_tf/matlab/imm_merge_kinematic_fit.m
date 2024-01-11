function [merged_S,merged_d2] = imm_merge_kinematic_fit(imm_S, imm_d2, imm)
%IMM_MERGE_KINEMATIC_FIT merges the statistical distance^2 and the
%corresponding covariance of multiple IMMs

merged_S = zeros(size(imm_S,1),size(imm_S,2));
merged_d2 = 0;

for imm_num = 1:size(imm_S,3)
    merged_S = merged_S + imm(imm_num).mu * imm_S(:,:,imm_num);
    merged_d2 = merged_d2 + imm(imm_num).mu * imm_d2(imm_num);
end

end

