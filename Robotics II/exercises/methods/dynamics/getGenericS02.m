
function [S0_,S_,S0v_] = getGenericS02(q,qd,svals)
% Very empiric (/messy) method to extract dynamic parameters from the Inertia
% matrix in robot dynamics. Based on grouping terms.

% q = vector of q parameters (symbolic)
% q = [q1, q2, ... , qn]
% exceptionTerms: literals or expressions that are known and should be kept
% outside the dynamic parameters, like links length. 
% exceptionTerms = [l(2)] 

% created by David Imbajoa (04/2020)
disp(">> Getting  Generics S0...")
[n,cols] = size(q); % square matrix
% allcomb=sort(nchoosek(1:n*n,n),2);
% secvals=diff(allcomb,1,2);
% nullcombs=~(sum(secvals,2)==(n-1));
% n=3
% allperm=perms(1:n)
% permlengh=size(allperm,1)
S0=sym('s',[n n],'real');
% S0_=S0;
S0_={};
S0v_={};
for iq=1:n
    S_=solve(S0*qd==0,S0(:,iq))
    S0_{iq}=subs(S0,S_);
    if(exist('svals'))
        S0v_{iq}=simplify(subs(S0_{iq},S0(:,(1:n)~=iq),svals));
    end
end
S_=S0;
S0_{:}
    if(exist('svals'))
        S0v_{:}
    end
end



