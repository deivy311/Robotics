function sincos_ = getShortNotation_SinCosV2(n,mode)

%GETSHORTNOTATION_SINCOS - computes the matrices to replace sine and cosine
% symbolic variables for a short notation ones, where:
% sin(qi) = s_i
% sin(qi + qj) = s_i_j
% This notation is widely used in robotics, where this terms are very usual
% in Jacobians and Tranformation matrices
%
% Syntax:  R = GETSHORTNOTATION_SINCOS(n)
%
% Inputs:
%    n - Number of joints in manipulator (in robotics)
%
% Outputs:
%    sincos_ - Object with the mapping of all possible combinations of
%    sines and cosines for n.
%
% Example: 
%    sincos_ = getShortNotation_SinCos(3)
%   
% See also: toShortNotation
% Author: David imbajoa - 
% email: pokra311@gmail.com -
% Last revision: 09-Jul-2019
if(~exist('mode','var'))
mode=1;
end
if(mode==1)
    q = sym('q', [n,1]);
else
    q = sym('q_', [n,1]);
end
combs=[];
combshort=[];
sym_short_c="c";
sym_short_s="s";

for i = 1:n
    tempcombs=unique(sort(nchoosek(repmat(1:n,1,i),i),2),'rows');
    sym_short_0=""+tempcombs(:,1);
    for j=2:i
        sym_short_0=sym_short_0+"_"+tempcombs(:,j);
    end
combs = [combs;sum(q(tempcombs),2)];
combshort = [combshort;sym_short_0];
end
qall_sincos_comb=[sin(combs);cos(combs)];
qall_sincos_comb_short=["s"+combshort;"c"+combshort];
sincos_.full=qall_sincos_comb;
sincos_.short=sym(qall_sincos_comb_short,'real');