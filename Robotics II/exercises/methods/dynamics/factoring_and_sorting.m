%Aurhor:    David Esteban Imnjoa Ruiz
%email:     imbajoaruiz.1922212@studenti.uniroma1.it
%functiontion to extract numeric factors and for sorting
function [terms_to_process_C,symbols_signs]=factoring_and_sorting(terms_to_process,extract_num)
%     temp_symbolic_term=sym('temp_symbolic_term_mul','real');
    symbols_signs=ones(size(terms_to_process));
    
    if(extract_num)
        for  i=1:length(terms_to_process)
            %temporal_syms_childs=cell2sym(children(terms_to_process(i)*temp_symbolic_term));
            temporal_syms_childs=factor(terms_to_process(i));
            temp_values_ind=isSymType(temporal_syms_childs,'real');   
            symbols_signs(i)=prod(temporal_syms_childs(temp_values_ind));
            terms_to_process(i)=simplify(terms_to_process(i)/symbols_signs(i));
        end
    end


    %getting just the unique symbols values
    [terms_to_process_C,terms_to_process_ia,terms_to_process_ic] = unique(terms_to_process,'legacy');
    symbols_signs=symbols_signs(terms_to_process_ia);

     %sorting symbols by lenght
    [B,I_sort]=sort(cellfun(@length,string(terms_to_process_C)),'ascend');
    symbols_signs=symbols_signs(I_sort);
    terms_to_process_C=terms_to_process_C(I_sort);
    
end


