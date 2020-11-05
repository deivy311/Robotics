
function [g_q, PE, U,g_q_short,PE_short,frictionviscous] = getGravityWithPc(p, q, l, m, d, g, isMotor)%,viscous_Friction)
% $Author: David Imbajoa $  $Date: 2020 $ $Revision: 0.1 $

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

[~,n]=size(p);

if ~exist('isMotor','var')
     % Default parameters
    isMotor = 0;
end   
frictionviscous=sym('fv',[n,1],'real');
% if ~exist('viscous_Friction')
% 
% end
qallcomb=[];


for ki=1:n%n for organizee similar parameters
    qallcomb=[qallcomb,sum(nchoosek(q,ki),2)']; 
    
end
qall_sincos_comb=[sin(qallcomb),cos(qallcomb)]
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


% [~ , TPartial] = getTransformationMatrix(dhTable,'alpha');
PE = 0;
% disp("if it doesn't work un comment the previous line and comment the next")
% PE = [0,0,0]';

for i = 1:n
    
    % potential energy is computed up to the i-th link. (Siciliano, Pag. 255)
    U{i} = simplify(-m(i)*g.'*p(:,i));
    U{i};
    % Accumulates the result.           
    PE = simplify(PE + U{i});
end

PE=optiomalSortD(PE,qall_sincos_comb)

g_q = jacobian(PE, q).';

    sincos_ = getShortNotation_SinCosV2(n);
    PE_short = toShortNotation(simplify(PE),sincos_)
    g_q_short = toShortNotation(simplify(g_q),sincos_)
end