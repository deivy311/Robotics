close all
clear all
clc
% close all
% % Lengths just to test
d1_=10;
d2_=10;
a2_=10;
% % De Luca's approach
 links = [
    Revolute('a',0, 'alpha',pi/2,'d', 1);
    Revolute('a',1, 'alpha',0,'d', 0);
%     Prismatic('qlim', [0 len1],'alpha',pi/2,'theta',pi);
%     Revolute('a', len1, 'alpha', 0 ,'d', 0);
    ];
% 
 c600 = SerialLink(links);
 c600.teach([pi/2 0],'view','y');
% c600.plot([0  pi/2 20 0],'view','x')