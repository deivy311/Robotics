function [Md,Mtd,qt] = getM_dot(M,q,qd,varnamedep)
% Very empiric (/messy) method to extract dynamic parameters from the Inertia
% matrix in robot dynamics. Based on grouping terms.

% q = vector of q parameters (symbolic)
% q = [q1, q2, ... , qn]
% exceptionTerms: literals or expressions that are known and should be kept
% outside the dynamic parameters, like links length. 
% exceptionTerms = [l(2)] 

if(~exist('varnamedep','var'))
    varnamedep='q';
end
% created by David Imbajoa (04/2020)
disp(">> Getting M dot. Might take a while...")
[n,cols] = size(M); % square matrix

t_name='t';
t=sym(t_name,'real');
[Mt,qt] = ToTimedep(M,n,t_name,varnamedep);
 Mtd=diff(Mt,t);
[Md,qall] = shortdiff(Mtd,sym2cell(qt),t,false,1);
if(exist('qd'))
    Md=subs(Md,qall(:,2),qd)
end

end