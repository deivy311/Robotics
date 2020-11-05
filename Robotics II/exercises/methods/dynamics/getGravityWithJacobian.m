
function [g_q, PE, U] = getGravityWithJacobian(dhTable, q, l, m, d, g, isMotor)

%corregir esta ecuación creo que está mal
% $Author: Andrés Arciniegas $  $Date: 2019/06/15 $ $Revision: 0.1 $

% Computes the Kinetic Energy of a robot based on the configuration of the
% DH table, with the Jacobian-based method, described in detail in (Siciliano, Pag. 254)
%
% Parameters:
% ------------------------------
% dhTable: Denavit-Hartenberg configuration table. 
%     Order:           alpha, a, d, theta
%     Example 2R Robot: [ 0, l1, 0, q1] 
%                       [ 0, l2, 0, q2]
%
% *** Symbolic variables: Vectors have to be represented as column vectors
% q: Joint values vector. Example: [q1,q2]
% qd: Joint velocities. Example: [qd1,qd2]
% l: link length Example: [l1,l2]
% m: mass of the link. Example: [m1,m2]
% I: Inertia values. Example: [I1,I2]
% d: Distance to the center of mass: Example: [d1,d2]
%
% *** Motor
% isMotor: (boolean) calculation of the motor kinetic energy?
% qdm: Joint velocities of the motor.
%
% *** returns
% PE: Accumulated Kinetic Energy from all joints
% U: Potential energy of each individual link.

disp('*** Computation of Potential Energy (Jacobian-based method) ')

[n,~]=size(dhTable);

if ~exist('isMotor','var')
     % Default parameters
    isMotor = 0;
end   

% If the 'Motor' option is used, some variables will be changed, and they
% should be added to the calling workspace.

% if isMotor==1 
%     if ~exist('dqm','var') || ~exist('Im','var')
%      % Default parameters
%       qdm = sym('qdm',[1 , n]).';
%     end
%     disp('*** KE of Motors chosen.')
%     disp('*** Remember to include the following changes:')
%     disp('*** I = Im; m = mm.')
%     I = sym('Im', [1,n]).';
%     m = sym('mm', [1,n]).';
% end   


[~ , TPartial] = getTransformationMatrix(dhTable,'alpha');
PE = 0;

for i = 1:n
    T_temp = eye(4);
    for j=1:i
        if j==i % Substitute the last link length by the distance to center of mass
            T_temp = T_temp*subs(TPartial{j},l(i),d(i))    ;
        else
            T_temp = T_temp*TPartial{j};
        end
    end
    
    p = simplify(T_temp(1:3,4));
    
    % potential energy is computed up to the i-th link. (Siciliano, Pag. 255)
    U{i} = simplify(m(i)*g.'*p);
    U{i};
    % Accumulates the result.           
    PE = simplify(PE + U{i});
end

g_q = jacobian(PE, q).';

end