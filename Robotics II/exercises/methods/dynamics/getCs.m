
function [C,cac,Csubs,S] = getCs(M,q,qd)
% Very empiric (/messy) method to extract dynamic parameters from the Inertia
% matrix in robot dynamics. Based on grouping terms.

% q = vector of q parameters (symbolic)
% q = [q1, q2, ... , qn]
% exceptionTerms: literals or expressions that are known and should be kept
% outside the dynamic parameters, like links length. 
% exceptionTerms = [l(2)] 

% created by David Imbajoa (04/2020)


disp(">> Getting  the Christoffel’sym. Might take a while...")
[n,cols] = size(M); % square matrix
qallcomb=[];


for ki=1:n%n for organizee similar parameters
    qallcomb=[qallcomb,sum(nchoosek(q,ki),2)']; 
    
end
qall_sincos_comb=[sin(qallcomb),cos(qallcomb)];
Msubs = M;
aa = sym('aa', [1,20]); % 20 is a random approximation. Depends on M. For RR Robot is around 5
dynamicParams = [];
nparams = 0; % Number of replacements
% assume(M,'real')

 for i=1:n
    CMi=simplify(jacobian(M(:,i),q));
    CM=simplify(diff(M,q(i)));
    C{i}=simplify((1/2)*(CMi+CMi'-CM));
    %     cac(i,1)=simplify(expand(qd'*C{i}*qd));
    cac(i,1)=qd'*C{i}*qd;
    
    %%reoordering
    cac(i,1)= optiomalSortD(cac(i,1),qall_sincos_comb);
    
    % symetric matrix lagrangian dynamics 1 page 22
    S(i,:)=optiomalSortD(qd'*C{i},qall_sincos_comb);%% 
    
    %
 end

Csubs=C;
end


