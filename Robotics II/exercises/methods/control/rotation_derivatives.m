%Aurhor:    David Esteban Imnjoa Ruiz
%email:     imbajoaruiz.1922212@studenti.uniroma1.it
%this functions has the intentio to split the matrix into
function [struct_sub_mat,params]=Adpatative_Control_parameters(submats,dinamics_parameters,known_params,aM)
    a  =expand(dinamics_parameters); %all parameteres available
    submats=expand(submats)
    a_k=expand(known_params);
    a_k_sym=sym('a_k',size(a_k),'real')
    a_k_sym_zeros=zeros(size(a_k))
    %sorting forlenght the symbolic values
    [B_u,I_u]=sort(cellfun(@length,string(a_k)),'descend')
    new_a=subs(a,a_k(I_u),a_k_sym(I_u))
    partial_a=subs(new_a,a_k_sym,a_k_sym_zeros)
    partial_a=partial_a(find(partial_a~=0))
    a_u_sym=sym('a_u',size(partial_a),'real')
    a_u_sym_zeros=zeros(size(a_u_sym))
    %sorting forlenght the symbolic values
    [B,I]=sort(cellfun(@length,string(partial_a)),'descend')
    new_a2=subs(new_a,partial_a(I),a_u_sym(I))
    %replacing in the matrix the new values
    
    %sorting forlenght the symbolic values
    [B_a,I_a]=sort(cellfun(@length,string(new_a2)),'descend')
    
    struct_sub_mat.new_submats=subs(submats,aM(I_a),new_a2(I_a))
    params.known_parameters_=a_k
    params.known_parameters=a_k_sym
    params.unknown_parameters_=a
    params.unknown_parameters_=partial_a
    params.unknown_parameters =a_u_sym
    
%     known_parameters_=
    struct_sub_mat.new_submats_u=subs(struct_sub_mat.new_submats,a_k_sym,a_k_sym_zeros)
    struct_sub_mat.new_submats_k=subs(struct_sub_mat.new_submats,a_u_sym,a_u_sym_zeros)
    
    %getting complete explicit matrices
    
    struct_sub_mat.new_submats_u_=subs(struct_sub_mat.new_submats_u,a_u_sym,partial_a)
    struct_sub_mat.new_submats_k_=subs(struct_sub_mat.new_submats_k,a_k_sym,a_k)
end
    
