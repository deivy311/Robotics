function [KE, T] = getKEwithJacobian(dhTable, sigma, q, qd, l, m, I, d, isMotor, qdm)
% $Author: Andrés Arciniegas $  $Date: 2019/06/13 $ $Revision: 0.1 $

% Computes the Kinetic Energy of a robot based on the configuration of the
% DH table, with the Jacobian-based method, described in detail in (Siciliano, Pag. 254)
%
% Parameters:
% ------------------------------
% dhTable: Denavit-Hartenberg configuration table. 
%     Order:           alpha, a, d, theta
%     Example 2R Robot: [ 0, l1, 0, q1] 
%                       [ 0, l2, 0, q2]
% sigma: Types of joints. 0:Revolute; 1:Prismatic
%       Example: [0,0] 
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
% KE: Accumulated Kinetic Energy from all joints
% T: Kinetic energy of each individual link.

disp('*** Computation of Kinetic Energy (Jacobian-based method) ')

n = length(sigma);
if ~exist('isMotor','var')
     % Default parameters
    isMotor = 0;
end   

% If the 'Motor' option is used, some variables will be changed, and they
% should be added to the calling workspace.

if isMotor==1 
    if ~exist('dqm','var') || ~exist('Im','var')
     % Default parameters
      qdm = sym('qdm',[1 , n]).';
    end
    disp('*** KE of Motors chosen.')
    disp('*** Remember to include the following changes:')
    disp('*** I = Im; m = mm.')
    I = sym('Im', [1,n]).';
    m = sym('mm', [1,n]).';
end   


[~ , TPartial] = getTransformationMatrix(dhTable,'alpha');
KE = 0;
for i = 1:n
    qd_ = qd; % Auxiliar qd variable
    if isMotor
        % This substitution by zero represents that motor is centered on the
        % origin of the frame. Also, that the KE of each motor is affected by
        % the linear velocity of the previous links.
        l_subs = 0;
        qd_ = subs(qd_,qd(i),qdm(i));
    else
        % This substitution represents that ith distance is taken from the
        % previous frame to the center of mass of link i.
        l_subs = d(i);
        
    end
    
    % Get rotation matrix
    R = TPartial{i}(1:3,1:3);
    [Jp, Jo] = getJacobian(subs(dhTable(1:i,:), l(i), l_subs), sigma, 'alpha');
    
    % Kinetic energy is computed up to the i-th link. (Siciliano, Pag. 254)
    T{i} = simplify(0.5*(m(i))*qd_(1:i).'*Jp.'*Jp*qd_(1:i) + ...
                    0.5*qd_(1:i).'*Jo.'*R*I(i)*R.'*Jo*qd_(1:i));
    
    % Accumulates the result.           
    KE = simplify(KE + T{i});
    
end

end