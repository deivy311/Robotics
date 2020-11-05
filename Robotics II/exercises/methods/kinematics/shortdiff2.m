function [converted,new_dep_var,convertedshort] = shortdiff2(toconvert,dep_var,inddep_var,on_short,grade,Nvartoreplace)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
syms('d',[1,grade],'real');
syms(inddep_var,'real');
% sym_dep_var=dep_var(inddep_var);
new_dep_var=[];
temp_dev=dep_var;
derivative_grade="d";
for i_sym=2:grade
    if(i_sym<3)
        derivative_grade=[derivative_grade;repmat('d',1,i_sym)];
    elseif (i_sym<4)
        derivative_grade=[derivative_grade;'td'];
    elseif (i_sym<5)
        derivative_grade=[derivative_grade;'qd'];
    else
        derivative_grade=[derivative_grade;repmat('d',1,i_sym)];
    end
        
end
for ider=1:size(dep_var,1)

    local_str_dep_var=replace(string(dep_var{ider,1}),"("+string(inddep_var)+")","");
    temp_dep_var=sym(local_str_dep_var,'real');
    %%new_dep_var=[new_dep_var;temp_dep_var,sym(local_str_dep_var+"_d",[1,grade],'real')];
    temp_strdot=split(local_str_dep_var,"_")';
    new_dep_var=[new_dep_var;temp_dep_var,sym(temp_strdot(:,1)+"_"+derivative_grade'+"ot_"+strjoin(temp_strdot(:,2:end),'_'),'real')];
 
end
for ider=1:grade
%     syms(dep_var(ider)+"("+string(inddep_var)+")")
%     assumeAlso(eval(dep_var(ider)),'real')
%     local_str_dep_var=replace(string(dep_var{ider,1}),"("+string(inddep_var)+")","");
%     temp_dep_var=sym(local_str_dep_var,'real');
%     new_dep_var=[new_dep_var;temp_dep_var,sym(local_str_dep_var+"_d",[1,grade],'real')];
    temp_dev=[temp_dev,diff(temp_dev(:,end),inddep_var)];
%     new_dep_var=[new_dep_var,sym(replace(string(sym_dep_var(ider,1)),"("+string(inddep_var)+")",""),'real')]
end
 converted =toconvert;
for ider=grade+1:-1:1
 converted =subs(converted,temp_dev(:,ider),new_dep_var(:,ider));
end
if(exist('Nvartoreplace'))
     converted =subs(converted,new_dep_var(:,2:end),Nvartoreplace);
end
convertedshort=[];
if(exist('on_short'))
    if(on_short)
    qtemp=sym('q',[10,1],'real')
    Njoints=length(has(new_dep_var(:,1),qtemp))
    sincos =getShortNotation_SinCos(Njoints);
     convertedshort =toShortNotation(converted,sincos);
    end

end
% derived = diff(toderive,dep_var);
% outputArg2 = subs()
end

