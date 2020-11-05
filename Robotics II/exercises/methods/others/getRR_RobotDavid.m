function [dhTable, Msubs, dynParams, a] = getRR_RobotDavid()
% Gets a prebuild model of the RR Robot
 %no funciona ver despues
sigma = [0;0] % 0: Revolute, 1: Prismatic
n = length(sigma); %Number of joints

q = sym('q', [n 1],'real');
alpha = sym('alpha', [n 1],'real');
theta = sym('theta', [n 1],'real');
d = sym('d', [n 1]);
a = sym('a', [n 1]);
qd = sym('qd',[n 1])
l = sym('l',[n 1]);
d = sym('d',[n 1]);
m = sym('m',[n 1]);
syms g0
g = [0 -g0 0].';

% % alpha,a, d, theta
% dhTable = [0 l(1) 0 q(1);
%            0 l(2) 0 q(2)];
% dhTable=[]
%  for i=1:n
%      if(sigma(i)==0||sigma(i)=='R')
%          dhTabletem=[alpha(i),a(i),d(i),q(i)];
%      else
%          dhTabletem=[alpha(i),a(i),q(i),theta(i)];
%      end
%  end
% Initialization
% I = sym(zeros(3,3,n));
% rc = [];
% for i = 1:n
%     rc = [rc [-l(i)+d(i); 0; 0]];
%     I(:,:,i) = diag(sym(strcat({'Ixx','Iyy','Izz'},int2str(i))));
% end
% 
% M = getM(dhTable, sigma, qd.', rc, m, I, 0);
% 
% cS = getcS(M,q,qd);
% gravity = Gravity(dhTable,g ,m,rc,q);
% 
% [Msubs, dynParams, a]= getDynamicParameters(M, q, []);

end