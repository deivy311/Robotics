%Aurhor:    David Esteban Imnjoa Ruiz
%email:     imbajoaruiz.1922212@studenti.uniroma1.it
%Function to create all possible combination of elements to extract from
%terms
function [independet_var_combs]=independet_var_combs_generator(q,n,no_sines)
    if(~exist('no_sines','var'))
        no_sines=false;
    end
    qallcomb=[];
    for ki=1:n%n for organizee similar parameters
        qallcomb=[qallcomb,sum(nchoosek(q,ki),2)'];
    end

%     qall_sincos_comb=[q',sin(qallcomb),cos(qallcomb)];
    qall_sincos_comb_2=[qallcomb];
    if(~no_sines)
        qall_sincos_comb_2=[qall_sincos_comb_2,sin(qallcomb),cos(qallcomb)];
    end
    independet_var_combs=[qall_sincos_comb_2,qall_sincos_comb_2.^2];
end