function sincos_ = getShortNotation_SinCos(n)

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
% Author: Andres Arciniegas - 
% email: arciandres@gmail.com -
% Last revision: 09-Jul-2019

q = sym('q', [n,1]);

sin_q = sym(zeros(n,1));
sin_qq = sym(zeros(n,n));
sin_qqq = sym(zeros(n,n,n));
sin_q_ = sym('s',[n,1]);
sin_qq_ = sym('s',[n,n]);
sin_qqq_ = sym('s',[n,n,n]);

cos_q = sym(zeros(n,1));
cos_qq = sym(zeros(n,n));
cos_qqq = sym(zeros(n,n,n));
cos_q_ = sym('c',[n,1]);
cos_qq_ = sym('c',[n,n]);
cos_qqq_ = sym('c',[n,n,n]);

for i = 1:n
    sin_q(i) = sin(q(i));
    cos_q(i) = cos(q(i));
    for j =i:n
        sin_qq(i,j) = sin(q(i)+q(j));
        cos_qq(i,j) = cos(q(i)+q(j));
        for k = j:n
            sin_qqq(i,j,k) = sin(q(i)+q(j)+q(k));
            cos_qqq(i,j,k) = cos(q(i)+q(j)+q(k));
        end
    end
end

% Delete zero elements.
sin_qq = reshape(sin_qq,n*n,1);
sin_qq_ = reshape(sin_qq_,n*n,1);
sin_qq_(find(sin_qq == 0)) = [];
sin_qq(find(sin_qq == 0)) = [];

sin_qqq = reshape(sin_qqq,n*n*n,1);
sin_qqq_ = reshape(sin_qqq_,n*n*n,1);
sin_qqq_(find(sin_qqq == 0)) = [];
sin_qqq(find(sin_qqq == 0)) = [];

cos_qq = reshape(cos_qq,n*n,1);
cos_qq_ = reshape(cos_qq_,n*n,1);
cos_qq_(find(cos_qq == 0)) = [];
cos_qq(find(cos_qq == 0)) = [];

cos_qqq = reshape(cos_qqq,n*n*n,1);
cos_qqq_ = reshape(cos_qqq_,n*n*n,1);
cos_qqq_(find(cos_qqq == 0)) = [];
cos_qqq(find(cos_qqq == 0)) = [];

sincos_ =   { sin_q, sin_q_; 
              sin_qq, sin_qq_; 
              sin_qqq, sin_qqq_; 
              cos_q, cos_q_;
              cos_qq, cos_qq_;
              cos_qqq, cos_qqq_ };