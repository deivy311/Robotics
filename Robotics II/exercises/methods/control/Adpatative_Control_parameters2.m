%Aurhor:    David Esteban Imnjoa Ruiz
%email:     imbajoaruiz.1922212@studenti.uniroma1.it
%this functions has the intentio to split the matrix into
function [struct_sub_mat,params]=Adpatative_Control_parameters2(submats,dinamics_parameters,known_params,aM)
%     a  =expand(dinamics_parameters); %all parameteres available
    submats=expand(submats)
%     a_k=expand(known_params);
%     a_k_sym=sym('a_k',size(a_k),'real')
%     a_k_sym_zeros=zeros(size(a_k))

    %extracting known terms terms
    [independet_var_combs_except_terms]=independet_var_combs_generator(known_params,length(known_params));
    compute_known_uknown_terms=true;
    [new_mat_struct]=getting_known_uknown_terms(dinamics_parameters,[],[],known_params,independet_var_combs_except_terms,'a',compute_known_uknown_terms);
%     %sorting forlenght the symbolic values
%     [B_u,I_u]=sort(cellfun(@length,string(a_k)),'descend')
%     new_a=subs(a,a_k(I_u),a_k_sym(I_u))
%     partial_a=subs(new_a,a_k_sym,a_k_sym_zeros)
%     partial_a=partial_a(find(partial_a~=0))
%     a_u_sym=sym('a_u',size(partial_a),'real')
%     a_u_sym_zeros=zeros(size(a_u_sym))
%     %sorting forlenght the symbolic values
%     [B,I]=sort(cellfun(@length,string(partial_a)),'descend')
%     new_a2=subs(new_a,partial_a(I),a_u_sym(I))
%     %replacing in the matrix the new values
%     
%     %sorting forlenght the symbolic values
%     [B_a,I_a]=sort(cellfun(@length,string(new_a2)),'descend')
%     
%     struct_sub_mat.new_submats=subs(submats,aM(I_a),new_a2(I_a))
    params.known_parameters_=new_mat_struct.new_dynamic_values_known
    params.known_parameters=new_mat_struct.new_a_known
    params.unknown_parameters_=new_mat_struct.new_dynamic_values_unknown
%     params.unknown_parameters_=new_mat_struct.new_a_unknown
    params.unknown_parameters =new_mat_struct.new_a_unknown
    
    params.all_parameters=[params.known_parameters;params.unknown_parameters]
    params.all_parameters_=[params.known_parameters_;params.unknown_parameters_]
%     known_parameters_=
%     struct_sub_mat.new_submats_u=subs(struct_sub_mat.new_submats,a_k_sym,a_k_sym_zeros)
%     struct_sub_mat.new_submats_k=subs(struct_sub_mat.new_submats,a_u_sym,a_u_sym_zeros)
%     
%     
    %getting complete explicit matrices
    
    struct_sub_mat.new_submats_u=subs(submats,aM,new_mat_struct.new_mat_unknown)
    struct_sub_mat.new_submats_k=subs(submats,aM,new_mat_struct.new_mat_known)
    
    struct_sub_mat.new_submats=subs(submats,aM,new_mat_struct.new_mat)

    %originalvalues
    struct_sub_mat.new_submats_u_   =subs(struct_sub_mat.new_submats_u,params.known_parameters,params.known_parameters_)
    struct_sub_mat.new_submats_k_   =subs(struct_sub_mat.new_submats_k,params.known_parameters,params.known_parameters_)
end
    
