function R = getRotAroundVector(r,angle)

%GETROTAROUNDVECTOR - Computes the rotation matrix aount a unit vector by a
%given angle. (Page 11, slides 07_PositionOrientation - De Luca)
% Syntax:  R = GETROTAROUNDVECTOR(angles_axis,angles_name)
%
% Inputs:
%    r - unit vector
%    angle - angle eta of rotation
%
% Outputs:
%    R - Generated rotation matrix 
%
% Example: 
%    unit_vector = [1/sqrt(3), -1/sqrt(3) 1/sqrt(3)].'
%    eta = deg2rad(-30)
%    R = getRotAroundVector(unit_vector,eta)
%   
% See also: getEulerRotationMatrices
% Author: Andres Arciniegas - 
% email: arciandres@gmail.com -
% Last revision: 05-Jul-2019

% Skew-symmetric matrix from vector
S_r = [0 -r(3) r(2) ; r(3) 0 -r(1) ; -r(2) r(1) 0 ];

R = r*r.' + (eye(3) - r*r.')*cos(angle) + S_r*sin(angle);