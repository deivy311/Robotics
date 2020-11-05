function [isPD] = isPositiveDefinite(M,method)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
isPD=false;
if(~exist('method'))
method=1;
end
if(method==1)
[~,r] = chol(M);
isPD=(r == 0 && rank(M) == size(M,1))
disp("Is positive definite")
else
 isPD=(  all(eig(M) > eps)) 
 disp("Is not positive definite")
end
end

