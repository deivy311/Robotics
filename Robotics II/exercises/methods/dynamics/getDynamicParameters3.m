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




