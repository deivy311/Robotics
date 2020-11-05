%aurhor David Esteban Imbajoa Ruiz
function [Pc,vc,w,T,Ti,M,VarShortRobot,Trans] = getGenericPC(sigmaD,l,q,qd,dc,m,I,methodD,dhTable,firstParam,rc,movingframes,angle_)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Gets a prebuild model of the RR Robot
if(~exist('movingframes'))
movingframes=false;

end
if(~exist('methodD'))
methodD=1;
end
if(~exist('firstParam'))
firstParam='alpha';
end
sigmaD=  upper(sigmaD);
sigmaD(find(sigmaD=='P'))=1 ;
sigmaD(find(sigmaD=='R'))=0 ;
% sigma = [0;0] % 0: Revolute, 1: Prismatic
n = size(sigmaD,2); %Number of joints
qallcomb=[];

if(~exist("angle_"))
      angle_=sym(zeros(1,n))
%     switch sigmaD(2,i)
%     case 1*'X'
%         angle_=0
%     case 1*'Y'
%         angle_=pi/2
%     case 1*'Z'
%         %angle_=0
%          ME = MException('case in Z axis not programmed!!1');
%          throw(ME)
%     otherwise
        %angle_=sym('alpha','real')
%     end
end
for ki=1:n%n for organizee similar parameters
    qallcomb=[qallcomb,sum(nchoosek(q,ki),2)']; 
    
end
qall_sincos_comb=[sin(qallcomb),cos(qallcomb)];
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
rcdefined=false;
if(~exist('rc'))
    rcdefined=true;
    rc = [];
end
if(~exist('I'))
    I={};
elseif(isempty(I))
    I= cell(1,n);
       
elseif(~iscell(I))
    I= sym2cell(I);

end
disp("si falla aquí está el error")
charaxis=["x","y","z"];

%
% [KE, T] = getKEwithJacobian(dhTable,sigma,q,qd,l,m,I,d,0);
% M = getInertiaMatrixFromKE(KE,qd);
% g_q = getGravityWithJacobian(dhTable,q,l,m,d,g,0)
% % M = getM(dhTable, sigma, qd, rc, m, I, 0);
% % M = getM(dhTable, sigma,q, qd, l, m, I,d, 0);%implementing doens't
% % working
% cS = getCs(M,q,qd);
% % gravity = Gravity(dhTable,g ,m,rc,q);%% i don't know
%
% [Msubs, dynParams, a]= getDynamicParameters(M, q, []);
if(methodD==1)
% [TTotal TPartial,RTotal,RPartial,PTotal, PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]= getTransformationMatrixConfig(sigmaD,l,q,dc,false);
[TTotal TPartial,RTotal,RPartial,PTotal, PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]= getTransformationMatrixConfigbyangle(sigmaD,l,q,dc,false,angle_);
disp("method with PC , pay attention!")
else
    %%
    disp("method with DH table, pay attention!")
[TTotal TPartial,RTotal,RPartial,PTotal, PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]= getTransformationMatrixConfigfromDH(dhTable,l,q,dc,false,firstParam);
ThetaPartial=[];
ThetaTotal=[];
end
for i = 1:n
    if(rcdefined||isempty(rc))
        rc = [rc [-l(i)+dc(i); 0; 0]];%%creoq eu cambia respecto al axis
    end
    I2(:,:,i) = diag(sym(strcat({'Ixx','Iyy','Izz'},int2str(i)),'real'));%no funciona
    if(isempty(I{i}))
        
        %         I{i}=sym("I"+i+repmat(charaxis,3,1)'+repmat(charaxis,3,1),'real');
        %         Ic{i}=simplify(RTotali{i}*I{i}*RTotali{i}');
        Ic{i}=I2(:,:,i);
    else
%         Ic{i}=simplify(RTotali{i}*I{i}*RTotali{i}');%% if it is alrady IC comment and decoment the line below
        Ic{i}=simplify(I{i});
        
    end
end
w0=[0,0,0]';
v0=[0,0,0]';
wlast=w0;
vlast=v0;
Zi=[0,0,1]';
Zi_=[0,0,1]';

%%modification added by David

% [KE, T] = getKEwithJacobianTrans(TPartial, sigmaD(1,:)*1, q, qd, l, m, I, dc,0)%%no funciona creo
% Pc(end,end)=ThetaTotal;
for i=1:n
%     Zi=RPartial{i}(:,end);%si falla edte es el error
    w_last_i=   [wlast+(1-sigmaD(1,i))*qd(i)*Zi];
    w(:,i)=simplify(RPartial{i}'*w_last_i);
    v(:,i)=simplify(RPartial{i}'*[vlast+(sigmaD(1,i)*1)*qd(i)*(Zi)+cross(w_last_i,PPartial{i})]);
    
    vlast=v(:,i);
    wlast=w(:,i);
    if(movingframes)
        disp("moving frames ONN pay attention!!")
%         vc(:,i)=collect(simplify(vlast+cross(wlast,rc(:,i))),[dc]);
        vc(:,i)=jacobian(Pc(:,i),q)*qd;

        
    else
        disp("moving frames OFF!!")
        vc(:,i)=jacobian(Pc(:,i),q)*qd;
        
    end
    %     vc(:,i)=optiomalSortD(vlast+cross(wlast,rc(:,i)),qall_sincos_comb);
%     Ic{i}=simplify(RTotali{i}*I{i}*RTotali{i}'); %ya definida

    %%konig theorem
%     Ti2(i)= simplify(0.5*m(i)*collect(simplify(expand(vc3(:,i)'*vc3(:,i))),[qd(1:n)])+0.5*simplify((w(:,i)'*Ic{i}*w(:,i))));%,[qd(1:2)]);
    %     Ti(i)= optiomalSortD(0.5*m(i)*vc(:,i)'*vc(:,i)+0.5*w(:,i)'*Ic{i}*w(:,i),qall_sincos_comb);%,[qd(1:2)]);
    Ti(i)= simplify(0.5*m(i)*collect(simplify(expand(vc(:,i)'*vc(:,i))),[qd(1:n)])+0.5*simplify((w(:,i)'*Ic{i}*w(:,i))));%,[qd(1:2)]);
    
end

T=sum(Ti);
M = simplify(getInertiaMatrixFromKE(T,qd));
%  [C,cac,Csubs,S] = getCs(M,q,qd);


    sincos_ = getShortNotation_SinCosV2(n);
    % % Jl_ = toShortNotation(Jl,sincos_)
    VarShortRobot.Pc = toShortNotation(simplify(Pc),sincos_);
    VarShortRobot.vc= toShortNotation(simplify(vc),sincos_);
    VarShortRobot.w= toShortNotation(simplify(w),sincos_);
    VarShortRobot.T= toShortNotation(simplify(T),sincos_);
    VarShortRobot.Ti= toShortNotation(simplify(Ti),sincos_);
    VarShortRobot.M= toShortNotation(simplify(M),sincos_);
    
            
    Trans.TTotal=TTotal;
    Trans.PTotali=PTotali;
    Trans.TPartial=TPartial;
    Trans.RTotal=RTotal;
    Trans.RPartial=RPartial;
    Trans.PTotal=PTotal;
    Trans.PPartial=PPartial;
    Trans.Pc=Pc;
    Trans.RTotali=RTotali;
    Trans.ThetaPartial=ThetaPartial;
    Trans.ThetaTotal=ThetaTotal;

end

