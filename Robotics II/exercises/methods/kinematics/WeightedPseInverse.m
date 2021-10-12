function [J_pseudo_w,J__pseudo,W,Wvars] = WeightedPseInverse(W,J,q,config)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if(isempty(W))
disp("W must be negative, pay attention")
config=config(1,:)*1;
n=length(config);
symW=sym('W',[n,n],'real');
Wlocal2=diag(symW).*(1-config)'+config';
W=eye(n).*Wlocal2
Wvars=symvar(W);
end
J_pseudo_w=(inv(W)*J'*inv(J*inv(W)*J'));

J__pseudo=pinv(J+ones(size(J))*realmin);
end

