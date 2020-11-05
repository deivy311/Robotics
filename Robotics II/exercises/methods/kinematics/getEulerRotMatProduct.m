function [R_total, R_partials] = getEulerRotMatProduct(angles_axis,angles_name)

%GETEULERROTMATPRODUCT - computes the product of Euler rotational matrices
% Syntax:  [R_total, R_partials] = getEulerRotMatProduct(angles_axis,angles_name)
%
% Inputs:
%    angles_axis - Literal of the axis of rotation (X Y or Z)
%    angles_name - In case the name of the angles have to be specified
%
% Outputs:
%    R_total - Cummulative product of the partial rotation matrices
%    R_partials - Set of rotational matrices used in the process
%
% Example: 
%    syms alpha beta gamma
%    [R_total, R_set] = getEulerRotMatProduct('YXY',[alpha, beta, gamma])
%
% See also: getEulerRotationMatrices
% Author: Andres Arciniegas - 
% email: arciandres@gmail.com -
% Last revision: 05-Jul-2019
angles_axis=upper(angles_axis);% added by david imbajoa
n = length(angles_axis);

switch nargin
    case 1 % Angles not defined, assing angle theta
        angles_name = sym('theta',[n,1]);
    case 2 
        assert(n == length(angles_name),"The number of elements must be the same in both arrays.");
end

% Get the default rotation matrices, and declare their angles
[Rx Ry Rz] = getEulerRotationMatrices() ;
a = sym('a', [1 3]);

R_partials = cell(n,1);
R = [];
R_total = eye(3);

for i = 1:length(angles_axis)
    switch angles_axis(i)
        case 'X'
           R = subs(Rx,angles_name(i));
        case 'Y'
           R = subs(Ry,angles_name(i));
        case 'Z'
           R = subs(Rz,angles_name(i));
    end
    R_total = R_total*R; % computes the accumulative product
    R_partials{i} = R;
end