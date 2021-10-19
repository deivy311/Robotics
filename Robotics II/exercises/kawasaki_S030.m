clear;clc; close all
sigma = [0;0;0;0;0;0] % 0: Revolute, 1: Prismatic
n = length(sigma); %Number of joints

q = sym('q', [n 1],'real');
qd = sym('qd',[n 1],'real');
qdd = sym('qdd',[n 1],'real');
d = sym('d', [n 1],'real');
a = sym('a', [n 1],'real');
l = sym('l',[n 1],'real');
d = sym('d',[n 1],'real');
m = sym('m',[n 1],'real');
I = sym('I',[n 1],'real');
syms g0
g = [0,0,-g0].' 
syms r a

% close all
% % Lengths just to test
d1_=10;
d2_=10;
a2_=10;
l_=ones(n,1)
a_=ones(n,1)
% % De Luca's approach
 links = [
    Revolute('a',0, 'alpha', pi/2 ,'d', l_(1));
    Revolute('a', l_(2), 'alpha',0,'d', 0 );
    Revolute('a', 0*l_(3), 'alpha', pi/2 ,'d',l_(3));
    Revolute('a', 0, 'alpha', pi/2 ,'d',l_(4)*3);
    Revolute('a', l_(5)*0, 'alpha', -pi/2 ,'d',l_(5)*0);
    Revolute('a', l_(6)*0, 'alpha', 0 ,'d', l_(6));

%     Prismatic('qlim', [0 len1],'alpha',pi/2,'theta',pi);
%     Revolute('a', len1, 'alpha', 0 ,'d', 0);
    ];
% 
 c600 = SerialLink(links);
 c600.teach([0 pi/2,pi/2*0,0,0,0],'view','x');
%   c600.teach([0 pi/2,pi/2*0,0,pi/2*0,0],'view','x');
%  c600.teach([0 pi/2,0,0],'view','x');
%  c600.teach([0 pi/2,0,0*pi/2,0,0],'view','x');