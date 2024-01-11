function [asso_list] = get_clean_asso_list() %#codegen

len_asso_list = 20;
num_imm_models = 3;

asso = struct('loc_nr',0,'y',zeros(3,1),'R',zeros(3,3), ...
              'PDH1',0,'PDH0',0,'potGhost',0,...
              'S',zeros(3,3,num_imm_models),'d2',zeros(num_imm_models,1));
          
asso_list = repmat(asso,50,len_asso_list);



