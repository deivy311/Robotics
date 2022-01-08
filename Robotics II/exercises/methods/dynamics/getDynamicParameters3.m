%Aurhor:    David Esteban Imnjoa Ruiz
%email:     imbajoaruiz.1922212@studenti.uniroma1.it
function [Msubs, dynamicParamsReturn_, a] = getDynamicParameters3(M,q, exceptionTerms,parametricmethod_)
M=expand(M);

if(~exist('known_terms','var'))
    known_terms=[];
end
if(~exist('exceptionTerms','var'))
    exceptionTerms=[]
end
disp(">> Getting dynamic coefficients. Might take a while...")
[n,cols] = size(M); % square matrix
Msubs = M;
dynamicParamsReturn_=[];
a=[];
global_var_name="a_b";
if(isempty(exceptionTerms))
    global_var_name="a";
end
[independet_var_combs]=independet_var_combs_generator(q,n);

[new_mat_structure]=getting_known_uknown_terms(Msubs,[],q,[],independet_var_combs,global_var_name); %apaprently it works,

%returning values
Msubs=new_mat_structure.new_mat;
dynamicParamsReturn_=new_mat_structure.new_dynamic_values_known;
a=new_mat_structure.new_a_known;

%returning values

if(~isempty(exceptionTerms))
    %extracting known terms terms
    no_sines=true;
    [independet_var_combs_except_terms]=independet_var_combs_generator(exceptionTerms,length(exceptionTerms),no_sines);
    [new_mat_known_structure]=getting_known_uknown_terms(dynamicParamsReturn_,[],exceptionTerms,[],independet_var_combs_except_terms,'a');
    new_mat_known_terms=new_mat_known_structure.new_mat;
    
    
    Msubs=subs(Msubs,a,new_mat_known_terms);
    dynamicParamsReturn_=new_mat_known_structure.new_dynamic_values_known;
    a=new_mat_known_structure.new_a_known;
end

end



% %functiontion to extract numeric factors and for sorting
% function [terms_to_process_C,symbols_signs]=factoring_and_sorting(terms_to_process,extract_num)
%     temp_symbolic_term=sym('temp_symbolic_term_mul','real');
%     symbols_signs=ones(size(terms_to_process));
%
%     if(extract_num)
%         for  i=1:length(terms_to_process)
%             temporal_syms_childs=cell2sym(children(terms_to_process(i)*temp_symbolic_term));
%             temp_values_ind=isSymType(temporal_syms_childs,'real');
%             symbols_signs(i)=prod(temporal_syms_childs(temp_values_ind));
%             terms_to_process(i)=simplify(terms_to_process(i)/symbols_signs(i));
%         end
%     end
%
%
%     %getting just the unique symbols values
%     [terms_to_process_C,terms_to_process_ia,terms_to_process_ic] = unique(terms_to_process,'legacy');
%     symbols_signs=symbols_signs(terms_to_process_ia);
%
%      %sorting symbols by lenght
%     [B,I_sort]=sort(cellfun(@length,string(terms_to_process_C)),'ascend')
%     symbols_signs=symbols_signs(I_sort);
%     terms_to_process_C=terms_to_process_C(I_sort);
%
% end


