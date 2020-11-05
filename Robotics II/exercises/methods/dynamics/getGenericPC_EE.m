function [f,J,V,J_ps,PCVar,PC_EEShort,VarShortRobot] = getGenericPC_EE(sigmaD,l,q,qd,dc,m,I,xyonly)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Gets a prebuild model of the RR Robot
sigmaD=  upper(sigmaD);
sigmaD(find(sigmaD=='P'))=1 ;
sigmaD(find(sigmaD=='R'))=0 ;
% sigma = [0;0] % 0: Revolute, 1: Prismatic
n = size(sigmaD,2); %Number of joints
qallcomb=[];


for ki=1:n%n for organizee similar parameters
    qallcomb=[qallcomb,sum(nchoosek(q,ki),2)']; 
    
end
% qall_sincos_comb=[sin(qallcomb),cos(qallcomb)];
qall_sincos_comb=[q',sin(qallcomb),cos(qallcomb)];

%
% q = sym('q', [n 1],'real');
% d = sym('d', [n 1],'real');
% a = sym('a', [n 1],'real');
% qd = sym('qd',[n 1],'real')
% l = sym('l',[n 1],'real');
% d = sym('d',[n 1],'real');
% m = sym('m',[n 1],'real');
% syms g0
% g = [0 -g0 0].';
% 
% % alpha,a, d, theta
% dhTable = [0 l(1) 0 q(1);
%            0 l(2) 0 q(2)];
%        
% % Initialization
% I = sym(zeros(3,3,n));
if(~exist('xyonly'))
xyonly=false;
end
sincos_ = getShortNotation_SinCosV2(n);

[Pc,vc,w,T,Ti,M,VarShortRobot,Trans] = getGenericPC(sigmaD,l,q,qd,dc,m,I);

% qtotal=sum(q'.*(1-sigmaD(1,:)));
% f=Pc(:,end);

f=Trans.PTotal(:,end);
PCVar.PTotali=Trans.PTotali(1:2,:);

if(xyonly)
f=f(1:2,:);

else
    f(end,:)=Trans.ThetaTotal;
    PCVar.PTotali(end+1,:)=cumsum(Trans.ThetaPartial)';

end
J=jacobian(f,q);  
V=optiomalSortD(J*qd,qall_sincos_comb);
J_ps=simplify(pinv(J));
PCVar.Pc=Pc;
PCVar.vc=vc;
PCVar.w=w;
PCVar.T=T;
PCVar.Ti=Ti;
PCVar.M=M;
PC_EE.f=f;
PC_EE.J=J;
PC_EE.J_ps=J_ps;
PC_EE.V=V;
PC_EEShort.f=toShortNotation(f,sincos_);
PC_EEShort.J=toShortNotation(J,sincos_);
PC_EEShort.V=toShortNotation(V,sincos_);
PC_EEShort.Pc=Pc;
PC_EEShort.PTotali=Trans.PTotali;
end

