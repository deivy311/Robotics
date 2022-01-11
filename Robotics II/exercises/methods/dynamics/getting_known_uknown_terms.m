%Aurhor:    David Esteban Imnjoa Ruiz
%email:     imbajoaruiz.1922212@studenti.uniroma1.it
%function to split termsfromknowns and unknows
function [new_mat_res]=getting_known_uknown_terms(old_terms,old_terms_a,except_terms,old_known_terms,independet_var_combs,global_var_name,compute_known_uknown_terms)
   compute_unknown_terms=false;
   independet_var_combs_known=independet_var_combs;
   local_var_name="";
   if(~exist('compute_known_uknown_terms','var'))
       compute_known_uknown_terms=false;
   end
   if(compute_known_uknown_terms)
       compute_unknown_terms=true;
       except_terms=old_known_terms;
       all_terms_list=symvar(old_terms);
       unknow_terms=all_terms_list(~ismember(all_terms_list,except_terms));
       local_var_name="_u";
       no_sines=true;
       [independet_var_combs_unknown]=independet_var_combs_generator(unknow_terms,length(unknow_terms),no_sines);
       independet_var_combs=[independet_var_combs,independet_var_combs_unknown];
   end
   new_mat_terms=old_terms;
   new_dynamic_values=[];
   new_vars=[];
   if(~isempty(except_terms))
    orginal_shape=size(old_terms);
    old_terms=expand(old_terms(:));
    random_sum_sym_term=sym('random_summation_term_for_proces_dv','real');
    random_mul_sym_term=sym('random_multiplciation_term_for_proces_dv','real');
    ind_terms_with_except_terms=has(old_terms,except_terms);
    terms_excep_to_process=old_terms(ind_terms_with_except_terms)+random_sum_sym_term;%+random_sum_sym_term^2+(random_sum_sym_term+random_mul_sym_term)*sin(random_sum_sym_term)^2;
    terms_no_excep_to_process=old_terms(~ind_terms_with_except_terms&(old_terms~=0));
%     terms_no_excep_to_process=terms_no_excep_to_process(find(terms_no_excep_to_process~=0));
    terms_excep_to_process=collect(terms_excep_to_process,independet_var_combs);
    terms_except_childrens=children(terms_excep_to_process);
    if(length(terms_excep_to_process)==1)
        terms_except_childrens_{1}=cell(terms_except_childrens);
        terms_except_childrens=terms_except_childrens_;
    end
    
    tot_except_terms_i_j_mat=[];
    tot_no_except_terms_i_j_mat=terms_no_excep_to_process;
    for i=1:size(terms_except_childrens,1)
        terms_except_children_i=cell2sym(terms_except_childrens{i});
        index_except_term_i=has(terms_except_children_i,except_terms);
        no_excep_terms_i=terms_except_children_i(~index_except_term_i);
        excep_terms_i=terms_except_children_i(index_except_term_i)*random_mul_sym_term;
        
        %adding no except terms
        tot_no_excep_terms_i=sum(no_excep_terms_i)-random_sum_sym_term;
        tot_excep_terms_i=expand(simplify(sum(excep_terms_i)/random_mul_sym_term));
        
        %attaching except terms
        if(compute_unknown_terms)
             if(tot_excep_terms_i~=1&tot_excep_terms_i~=0)
                    l_no_except_terms=[];
                    %if the process doesn't workwell jsut comment this line
                    [l_except_terms,l_no_except_terms]=recursive_self_term_isolation(tot_excep_terms_i,except_terms,random_sum_sym_term,random_mul_sym_term);
                    tot_except_terms_i_j_mat=[tot_except_terms_i_j_mat;l_except_terms];
                    tot_no_except_terms_i_j_mat=[tot_no_except_terms_i_j_mat;l_no_except_terms];%check_this
            end
        end
        if(tot_no_excep_terms_i~=1&tot_no_excep_terms_i~=0)
            tot_no_except_terms_i_j_mat=[tot_no_except_terms_i_j_mat;tot_no_excep_terms_i];
        end
        for j=1:size(excep_terms_i,2)
            excep_terms_i_j=excep_terms_i(j);
            terms_except_childrens_i_j=factor(excep_terms_i_j);%cell2sym(children(excep_terms_i_j));
            index_except_term_i_j=has(terms_except_childrens_i_j,except_terms);
            no_excep_terms_i_j=terms_except_childrens_i_j(~index_except_term_i_j);
            excep_terms_i_j=terms_except_childrens_i_j(index_except_term_i_j)*1;

            %recovering known terms
            tot_except_terms_i_j=prod(excep_terms_i_j);
            tot_no_except_terms_i_j=prod(no_excep_terms_i_j)/random_mul_sym_term;

            %attaching except terms
            if(compute_unknown_terms)
                if(tot_except_terms_i_j~=1&tot_except_terms_i_j~=0)
                    tot_except_terms_i_j_mat=[tot_except_terms_i_j_mat;((tot_except_terms_i_j))];
                end
            end
            if(true)
                %attaching no except terms
                if(tot_no_except_terms_i_j~=1&tot_no_except_terms_i_j~=0)
                    tot_no_except_terms_i_j_mat=[tot_no_except_terms_i_j_mat;((tot_no_except_terms_i_j))];
                end
            end
        end
    end
    %sorting for lenght the symbolic values and recovering numeric  factors
    if(compute_unknown_terms)
        [tot_except_terms_i_j_mat_C,tot_except_terms_i_j_mat_signs]=factoring_and_sorting(tot_except_terms_i_j_mat,true);
        [new_mat_terms2,new_dynamic_values_known2,new_vars_known2]=smart_subs(old_terms,tot_except_terms_i_j_mat_C,global_var_name+"_k",independet_var_combs_unknown);
        old_terms=new_mat_terms2;
    end
    [tot_no_except_terms_i_j_mat_C,tot_no_except_terms_i_j_mat_signs]=factoring_and_sorting(tot_no_except_terms_i_j_mat,true);
    
    [new_mat,new_dynamic_values_unknown,new_a_unknown]=smart_subs(old_terms,tot_no_except_terms_i_j_mat_C,global_var_name+local_var_name,independet_var_combs_known);
    %[new_mat_known,new_dynamic_values_known,new_a__known]=smart_subs(old_terms,tot_no_except_terms_i_j_mat_C,global_var_name+local_var_name,independet_var_combs_known);
    new_mat_reshaped=reshape(new_mat,orginal_shape);
    new_mat_res.new_mat=new_mat_reshaped;
    new_mat_res.new_dynamic_values_unknown=new_dynamic_values_unknown;
    new_mat_res.new_a_unknown=new_a_unknown;

    if(compute_unknown_terms)
        new_mat_res.new_mat_known=simplify(subs(new_mat_reshaped,new_a_unknown,zeros(size(new_a_unknown))));
        new_mat_res.new_mat_unknown=simplify(new_mat_reshaped-new_mat_res.new_mat_known);

        new_mat_res.new_dynamic_values_known=new_dynamic_values_known2;
        new_mat_res.new_a_known=new_vars_known2;
    end
end
end
function [except_terms_buffer,no_except_terms_buffer]=recursive_self_term_isolation(tot_excep_terms,except_terms,random_sum_sym_term,random_mul_sym_term)
        tot_excep_terms=collect(tot_excep_terms,except_terms);
        temp_children=children((tot_excep_terms)+random_sum_sym_term)*random_mul_sym_term;
        ind_temp_children=has(temp_children,except_terms);
        new_temp_except_terms=temp_children(ind_temp_children);
        new_temp_no_except_terms2=sum(temp_children(~ind_temp_children)/random_mul_sym_term)-random_sum_sym_term;
        
        except_terms_buffer=[];
        no_except_terms_buffer=[];

        if(new_temp_no_except_terms2~=1&new_temp_no_except_terms2~=0)
            no_except_terms_buffer=new_temp_no_except_terms2;
        end

%         new_no_except_terms=prod(new_mul_children(~ind_temp_children))/random_mul_sym_term;
%         no_except_terms_buffer=[no_except_terms_buffer;new_no_except_terms];

        length_sums=size(new_temp_except_terms,2);
        temp_term_except_part=sum(new_temp_except_terms/random_mul_sym_term);
        dependencies_ecept_part=symvar(temp_term_except_part);
        remaining_non_except_vars=setdiff(dependencies_ecept_part,except_terms);
        if(isempty(remaining_non_except_vars))
            except_terms_buffer=[except_terms_buffer;temp_term_except_part];
        else
        
        
        for ij=1:length_sums
            
            tot_excep_terms_mul=new_temp_except_terms(1,ij);
            new_mul_children=children(tot_excep_terms_mul)*sym(1);
            ind_temp_children=has(new_mul_children,except_terms);
            new_temp_mul_term=new_mul_children(ind_temp_children);
            
            new_no_except_terms=prod(new_mul_children(~ind_temp_children))/random_mul_sym_term;
            no_except_terms_buffer=[no_except_terms_buffer;new_no_except_terms];

            sym_dependecies=symvar(new_temp_mul_term);
            new_set_terms_eval= setdiff(sym_dependecies,except_terms);
            if (length(new_set_terms_eval)>0)
                
                [l_except_terms,l_no_except_terms]=recursive_self_term_isolation(new_temp_mul_term,except_terms,random_sum_sym_term,random_mul_sym_term);
                except_terms_buffer=[except_terms_buffer;l_except_terms];
                no_except_terms_buffer=[no_except_terms_buffer;l_no_except_terms];
            else
                new_no_except_terms_mul=prod(new_mul_children(ind_temp_children));
                except_terms_buffer=[except_terms_buffer;new_no_except_terms_mul];
            end
        end
        end


end
% %Aurhor:    David Esteban Imnjoa Ruiz
% %email:     imbajoaruiz.1922212@studenti.uniroma1.it
% %function to split termsfromknowns and unknows
% function [new_mat_res]=getting_known_uknown_terms(old_terms,old_terms_a,except_terms,old_known_terms,independet_var_combs,global_var_name,compute_known_uknown_terms)
%    compute_unknown_terms=false;
%    independet_var_combs_known=independet_var_combs;
%    local_var_name="";
%    if(~exist('compute_known_uknown_terms','var'))
%        compute_known_uknown_terms=false;
%    end
%    if(compute_known_uknown_terms)
%        compute_unknown_terms=true;
%        except_terms=old_known_terms;
%        all_terms_list=symvar(old_terms);
%        unknow_terms=all_terms_list(~ismember(all_terms_list,except_terms));
%        local_var_name="_k";
%        no_sines=true;
%        [independet_var_combs_unknown]=independet_var_combs_generator(unknow_terms,length(unknow_terms),no_sines);
%        independet_var_combs=[independet_var_combs,independet_var_combs_unknown];
%    end
%    new_mat_terms=old_terms;
%    new_dynamic_values=[];
%    new_vars=[];
%    if(~isempty(except_terms))
%     orginal_shape=size(old_terms);
%     old_terms=expand(old_terms(:));
%     random_sum_sym_term=sym('random_summation_term_for_proces_dv','real');
%     random_mul_sym_term=sym('random_multiplciation_term_for_proces_dv','real');
%     ind_terms_with_except_terms=has(old_terms,except_terms);
%     terms_excep_to_process=old_terms(ind_terms_with_except_terms)+random_sum_sym_term;%+random_sum_sym_term^2+(random_sum_sym_term+random_mul_sym_term)*sin(random_sum_sym_term)^2;
%     terms_no_excep_to_process=old_terms(~ind_terms_with_except_terms&(old_terms~=0));
% %     terms_no_excep_to_process=terms_no_excep_to_process(find(terms_no_excep_to_process~=0));
%     terms_excep_to_process=collect(terms_excep_to_process,independet_var_combs);
%     terms_except_childrens=children(terms_excep_to_process);
%     if(length(terms_excep_to_process)==1)
%         terms_except_childrens_{1}=cell(terms_except_childrens);
%         terms_except_childrens=terms_except_childrens_;
%     end
%     
%     tot_except_terms_i_j_mat=[];
%     tot_no_except_terms_i_j_mat=terms_no_excep_to_process;
%     for i=1:size(terms_except_childrens,1)
%         terms_except_children_i=cell2sym(terms_except_childrens{i});
%         index_except_term_i=has(terms_except_children_i,except_terms);
%         no_excep_terms_i=terms_except_children_i(~index_except_term_i);
%         excep_terms_i=terms_except_children_i(index_except_term_i)*random_mul_sym_term;
%         
%         %adding no except terms
%         tot_no_excep_terms_i=sum(no_excep_terms_i)-random_sum_sym_term;
%         tot_excep_terms_i=expand(simplify(sum(excep_terms_i)/random_mul_sym_term));
%         
%         %attaching except terms
%         if(compute_unknown_terms)
%              if(tot_excep_terms_i~=1&tot_excep_terms_i~=0)
%                 tot_except_terms_i_j_mat=[tot_except_terms_i_j_mat;tot_excep_terms_i];
%             end
%         end
%         if(tot_no_excep_terms_i~=1&tot_no_excep_terms_i~=0)
%             tot_no_except_terms_i_j_mat=[tot_no_except_terms_i_j_mat;tot_no_excep_terms_i];
%         end
%         for j=1:size(excep_terms_i,2)
%             excep_terms_i_j=excep_terms_i(j);
%             terms_except_childrens_i_j=factor(excep_terms_i_j);%cell2sym(children(excep_terms_i_j));
%             index_except_term_i_j=has(terms_except_childrens_i_j,except_terms);
%             no_excep_terms_i_j=terms_except_childrens_i_j(~index_except_term_i_j);
%             excep_terms_i_j=terms_except_childrens_i_j(index_except_term_i_j)*1;
% 
%             %recovering known terms
%             tot_except_terms_i_j=prod(excep_terms_i_j);
%             tot_no_except_terms_i_j=prod(no_excep_terms_i_j)/random_mul_sym_term;
% 
%             %attaching except terms
%             if(compute_unknown_terms)
%                 if(tot_except_terms_i_j~=1&tot_except_terms_i_j~=0)
%                     tot_except_terms_i_j_mat=[tot_except_terms_i_j_mat;((tot_except_terms_i_j))];
%                 end
%             end
%             if(true)
%                 %attaching no except terms
%                 if(tot_no_except_terms_i_j~=1&tot_no_except_terms_i_j~=0)
%                     tot_no_except_terms_i_j_mat=[tot_no_except_terms_i_j_mat;((tot_no_except_terms_i_j))];
%                 end
%             end
%         end
%     end
%     %sorting for lenght the symbolic values and recovering numeric  factors
%     if(compute_unknown_terms)
%         [tot_except_terms_i_j_mat_C,tot_except_terms_i_j_mat_signs]=factoring_and_sorting(tot_except_terms_i_j_mat,true);
%         [new_mat_terms,new_dynamic_values_unknown,new_vars_unknown]=smart_subs(old_terms,tot_except_terms_i_j_mat_C,global_var_name+"_u",independet_var_combs_unknown);
%         old_terms=new_mat_terms;
%     end
%     [tot_no_except_terms_i_j_mat_C,tot_no_except_terms_i_j_mat_signs]=factoring_and_sorting(tot_no_except_terms_i_j_mat,true);
%     
%     [new_mat,new_dynamic_values_known,new_a_known]=smart_subs(old_terms,tot_no_except_terms_i_j_mat_C,global_var_name+local_var_name,independet_var_combs_known);
%     %[new_mat_known,new_dynamic_values_known,new_a__known]=smart_subs(old_terms,tot_no_except_terms_i_j_mat_C,global_var_name+local_var_name,independet_var_combs_known);
%     new_mat_reshaped=reshape(new_mat,orginal_shape);
%     new_mat_res.new_mat=new_mat_reshaped;
%     new_mat_res.new_dynamic_values_known=new_dynamic_values_known;
%     new_mat_res.new_a_known=new_a_known;
% 
%     if(compute_unknown_terms)
%         new_mat_res.new_mat_known=simplify(new_mat_reshaped-subs(new_mat_reshaped,new_a_known,zeros(size(new_a_known))));
%         new_mat_res.new_mat_unknown=simplify(new_mat_reshaped-new_mat_res.new_mat_known);
% 
%         new_mat_res.new_dynamic_values_unknown=new_dynamic_values_unknown;
%         new_mat_res.new_a_unknown=new_vars_unknown;
%     end
% end
% end
