function [varconverted,qt] = ToTimedep(var2convert,n,var_ind,var_dep)
% var_ind = 't';
% var_dep = 'q';
var_set =[]
syms(var_ind)
for i=1:n
   var_dep_str = strcat(var_dep,num2str(i),'(',var_ind,')');
   syms(var_dep_str);
   vaarb1=str2sym(var_dep_str);
%    assume(vaarb1,'real')

   var_set = [var_set, vaarb1]; % appends the variable to the set
end

% syms t t_
t=sym(var_ind,'real');
dep1=sym(var_dep,[n,1],'real');
 assume(var_set,'real');

qt = var_set';
% qdt = diff(qt,t)
varconverted=subs(var2convert,dep1,qt);
end

