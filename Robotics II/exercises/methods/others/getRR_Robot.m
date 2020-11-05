function [dhTable, Msubs, dynParams, a] = getRR_Robot()
% Gets a prebuild model of the RR Robot

sigma = [0;0] % 0: Revolute, 1: Prismatic
n = length(sigma); %Number of joints

q = sym('q', [n 1],'real');
d = sym('d', [n 1],'real');
a = sym('a', [n 1],'real');
qd = sym('qd',[n 1],'real')
l = sym('l',[n 1],'real');
d = sym('d',[n 1],'real');
m = sym('m',[n 1],'real');
syms g0
g = [0 -g0 0].';

% alpha,a, d, theta
dhTable = [0 l(1) 0 q(1);
           0 l(2) 0 q(2)];
       
% Initialization
I = sym(zeros(3,3,n));
rc = [];
for i = 1:n
    rc = [rc [-l(i)+d(i); 0; 0]];
    I(:,:,i) = diag(sym(strcat({'Ixx','Iyy','Izz'},int2str(i))));
end

[KE, T] = getKEwithJacobian(dhTable,sigma,q,qd,l,m,I,d,0);
M = getInertiaMatrixFromKE(KE,qd);
g_q = getGravityWithJacobian(dhTable,q,l,m,d,g,0)
% M = getM(dhTable, sigma, qd, rc, m, I, 0);
% M = getM(dhTable, sigma,q, qd, l, m, I,d, 0);%implementing doens't
% working
cS = getCs(M,q,qd);
% gravity = Gravity(dhTable,g ,m,rc,q);%% i don't know

[Msubs, dynParams, a]= getDynamicParameters(M, q, []);

end