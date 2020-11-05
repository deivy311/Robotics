function obj_out = toLongNotation(obj, sincos_)
%TOLONGTNOTATION - : puts an symbolic matrix object into long notation
%format
% sin(qi) = s_i
% sin(qi + qj) = s_i_j
% This notation is widely used in robotics, where this terms are very usual
% in Jacobians and Tranformation matrices
%
% Syntax:  R = TOLONGNOTATION(n)
%
% Inputs:
%    obj - Object to put in short notation format
%    sincos_ - mapping gotten from getShortNotation_SinCos method
%
% Outputs:
%    obj_out - Object in short notation format
%
% Example: 
%    ... (dhTable calculation)
%    [Jl, Jw] = getJacobian(dhTable, sigma, 'alpha')
%    sincos_ = TOLONGNOTATION(3)
%    obj_out = getShortNotation_SinCos(obj, sincos_)
%   
% See also: getShortNotation_SinCos toShortNotation getJacobian getTransformationMatrix
% Author: Andres Arciniegas - 
% email: arciandres@gmail.com -
% Last revision: 09-Jul-2019

obj_out = obj;
for i = 1:length(sincos_)
    obj_out = subs(obj_out,sincos_{i,2}, sincos_{i,1});
end