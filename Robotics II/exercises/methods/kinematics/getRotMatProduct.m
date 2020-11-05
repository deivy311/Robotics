function [R_total, R_partials] = getRotMatProduct(angles_axis,type,angles_name)

%GETROTMATPRODUCT - computes the product of Euler rotational matrices
% Syntax:  [R_total, R_partials] = GETROTMATPRODUCT(angles_axis,angles_name)
%
% Inputs:
%    angles_axis - Literal of the axis of rotation (X Y or Z)
%    angles_name - In case the name of the angles have to be specified
%    type = 0:Euler ; 1:roll-pitch-yaw
%
% Outputs:
%    R_total - Cummulative product of the partial rotation matrices
%    R_partials - Set of rotational matrices used in the process
%
% Example: 
%    syms alpha beta gamma
%    [R_total, R_set] = getRotMatProduct('YXY',0,[alpha, beta, gamma])
%
% See also: getEulerRotationMatrices
% Author: Andres Arciniegas - 
% email: arciandres@gmail.com -
% Last revision: 05-Jul-2019

n = length(angles_axis);

switch nargin
    case 1 % Angles not defined, assing angle theta
        angles_name = sym('theta',[n,1]);
    case 2 
        assert(n == length(angles_name),"The number of elements must be the same in both arrays.");
end

% Get the default rotation matrices, and declare their angles
[Rx Ry Rz] = getEulerRotationMatrices() ;

R_partials = cell(n,1);
R = [];
R_total = eye(3);

for i = 1:n
    j = n*type + (1-2*type)*i + 1*type; %-1 when 1; 1 when zero % This reversed the sequence. Ex. i=1,2,3 -> j=3,2,1
    switch angles_axis(j)
        case 'X'
           R = subs(Rx,angles_name(j));
        case 'Y'
           R = subs(Ry,angles_name(j));
        case 'Z'
           R = subs(Rz,angles_name(j));
    end
    R_total = R_total*R; % computes the accumulative product
    R_partials{i} = R;
end