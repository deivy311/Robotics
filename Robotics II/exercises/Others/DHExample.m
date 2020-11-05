clear; clc;
% From Denavit-Hartemberg table to Homogeneous transform Matrix

qnum = 7; %Number of joints

%Sym values for depicting undefined variables in result. They can be
%numbers
q = sym('q', [1 qnum]);
d = sym('d', [1 qnum]);
a = sym('a', [1 qnum]); 

%%
%Denavit-Hartemberg table - definition of robot.
%Must be defined in the order a - alpha - d - theta

%Spherical robot. EDX Robotics course Analytical IK, Spherical Robot Part 1

dhTable1 = [ 0 -pi/2 0 q(1);
             0  pi/2 0 q(2);
             0  0  q(3) 0;
             0  0    0 q(4);];
    
T1 = getTranslationMatrix(dhTable1)