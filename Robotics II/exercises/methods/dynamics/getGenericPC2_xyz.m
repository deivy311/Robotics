%aurhor David Esteban Imbajoa Ruiz
function [Pc,vc,w,T,Ti,M,VarShortRobot,Trans] = getGenericPC2_xyz(z)
%sigmaD,l,q,qd,dc,m,I,methodD,dhTable,firstParam,rc,movingframes,angle_)


sigmaD=z.sigmaD;
l=z.l;
q=z.q;
qd=z.q_dot;
dc=z.dc;
m=z.m;
I=z.I;
methodD=z.methodD;
dhTable=z.dhTable;
firstParam=z.firstParam;
rc=z.rc;
movingframes=z.movingframes;
angle_=z.angle_;
opt_expr=z.opt_expr;
prismatic_CoM_method=z.prismatic_CoM_method;


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
% rcdefined=z.rcdefined;
% if(~exist('rc'))
%     rcdefined=true;
%     rc = [];
% end
 if(isempty(rc))
     rcdefined=true
     z.rcdefined=true
 end
 if(~exist('I'))
     I={};
     if(z.ismotor)
         z.I_m={};
     end
 elseif(isempty(I))
     I= cell(1,n);
     if(z.ismotor)
         z.I_m= cell(1,n);
     end
 elseif(~iscell(I))
     I= sym2cell(I);
     if(z.ismotor)
         z.I_m= sym2cell(z.I_m);
     end
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
    
    [TTotal TPartial,RTotal,RPartial,PTotal, PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]= getTransformationMatrixConfigbyangle(sigmaD,l,q,dc,false,angle_,prismatic_CoM_method,z.xyx_offset,z);
    
    disp("method with PC , pay attention!")
else
    
    disp("method with DH table, pay attention!")
    [TTotal TPartial,RTotal,RPartial,PTotal, PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]= getTransformationMatrixConfigfromDH(dhTable,l,q,dc,false,firstParam);
    
    ThetaPartial=[];
    ThetaTotal=[];
end

if(z.ismotor)
    Pc_m=sym(zeros(size(Pc)));
%     TTotal =subs(TTotal,opt_expr{1},opt_expr{2}) ;
%     RTotal=subs(RTotal,opt_expr{1},opt_expr{2}) ;
%     PTotal=subs(PTotal,opt_expr{1},opt_expr{2});
%     Pc=subs(Pc,opt_expr{1},opt_expr{2}) ;
%     ThetaTotal=subs(ThetaTotal,opt_expr{1},opt_expr{2}) ;
%     ThetaPartial=subs(ThetaPartial,opt_expr{1},opt_expr{2}) ;
%     PTotali=subs(PTotali,opt_expr{1},opt_expr{2}) ;
    
    %dynamics parameters, not well tested
    
    for ird=1:n
        vars2replace_motor =[l(ird),q(ird),qd(ird),dc(ird)];
        vars2replace_motor_=[l(ird),z.q_m(ird),z.q_dot_m(ird),0];
        Pc_m(:,ird)=subs(Pc(:,ird),vars2replace_motor,vars2replace_motor_) ;
%         TPartial{ird}=subs(TPartial{ird},opt_expr{1},opt_expr{2}) ;
%         RPartial{ird}=subs(RPartial{ird},opt_expr{1},opt_expr{2});
%         PPartial{ird}=subs(PPartial{ird},opt_expr{1},opt_expr{2}) ;
%         RTotali{ird}=subs(RTotali{ird},opt_expr{1},opt_expr{2}) ;
        
        
    end
end    
if(~isempty(opt_expr))
    TTotal =subs(TTotal,opt_expr{1},opt_expr{2}) ;
    RTotal=subs(RTotal,opt_expr{1},opt_expr{2}) ;
    PTotal=subs(PTotal,opt_expr{1},opt_expr{2});
    Pc=subs(Pc,opt_expr{1},opt_expr{2}) ;
    ThetaTotal=subs(ThetaTotal,opt_expr{1},opt_expr{2}) ;
    ThetaPartial=subs(ThetaPartial,opt_expr{1},opt_expr{2}) ;
    PTotali=subs(PTotali,opt_expr{1},opt_expr{2}) ;
    
    %dynamics parameters, not well tested
    m=subs(m,opt_expr{1},opt_expr{2}) ;
    z.m=m;
    for ird=1:n
        TPartial{ird}=subs(TPartial{ird},opt_expr{1},opt_expr{2}) ;
        RPartial{ird}=subs(RPartial{ird},opt_expr{1},opt_expr{2});
        PPartial{ird}=subs(PPartial{ird},opt_expr{1},opt_expr{2}) ;
        RTotali{ird}=subs(RTotali{ird},opt_expr{1},opt_expr{2}) ;
        I{ird}=subs(I{ird},opt_expr{1},opt_expr{2}) ;%% test it
        
    end
    
end
for i = 1:n
%     if(rcdefined||isempty(rc))
    if(rcdefined)
        rc = [rc [-l(i)+dc(i); 0; 0]];%%creoq eu cambia respecto al axis
    end
    I2(:,:,i) = diag(sym(strcat({'Ixx_','Iyy_','Izz_'},int2str(i)),'real'));%no funciona
    if(isempty(I{i}))
        
        %         I{i}=sym("I"+i+repmat(charaxis,3,1)'+repmat(charaxis,3,1),'real');
        %         Ic{i}=simplify(RTotali{i}*I{i}*RTotali{i}');
        Ic{i}=I2(:,:,i);
    else
%         Ic{i}=simplify(RTotali{i}*I{i}*RTotali{i}');%% if it is alrady IC comment and decoment the line below
        Ic{i}=simplify(I{i});
        if(z.ismotor)
            z.Ic_m{i}=simplify(z.I_m{i});
        end
    end
end
w0=[0,0,0]';
v0=[0,0,0]';
wlast=w0;
vlast=v0;
Zi=[0,0,1]';

%motor %%%%%%%%%%%%%%
w_m_0=[0,0,0]';
v_m_0=[0,0,0]';
wlast_m=w0;
vlast_m=v0;
% Zi=[0,0,1]';
%%%%%%%%%%%%%%%%%
Zi_=[0,0,1]';

%%modification added by David

% [KE, T] = getKEwithJacobianTrans(TPartial, sigmaD(1,:)*1, q, qd, l, m, I, dc,0)%%no funciona creo
% Pc(end,end)=ThetaTotal;
for i=1:n
%     Zi=RPartial{i}(:,end);%si falla edte es el error
    if(z.global_q_reference)%% test it !!!!
        %%using global reference for q!! pay attention
        w_last_i=   [wlast+(1-sigmaD(1,i))*z.q_dot_global(i)*Zi];
        if(z.ismotor)
            w_last_m_i=   [wlast_m+(1-sigmaD(1,i))*z.q_dot_global_m(i)*Zi];
        end
    else
        w_last_i=   [wlast+(1-sigmaD(1,i))*qd(i)*Zi];
        if(z.ismotor)
            w_last_m_i=   [wlast_m+(1-sigmaD(1,i))*z.q_dot_m(i)*Zi];
        end
    end
    w(:,i)=simplify(RPartial{i}'*w_last_i);
    v(:,i)=simplify(RPartial{i}'*[vlast+(sigmaD(1,i)*1)*qd(i)*(Zi)+cross(w_last_i,PPartial{i})]);
    
    if(z.ismotor)
        
        %         w_m(:,i)=simplify(RPartial{i}'*w_last_m_i);
        %         v_m(:,i)=simplify(RPartial{i}'*[vlast_m+(sigmaD(1,i)*1)*z.q_dot_m(i)*(Zi)+cross(w_last_m_i,PPartial{i})]);
        
        vars2replace_motor =[l(i),q(i),qd(i),dc(i)];
        vars2replace_motor_=[l(i),z.q_m(i),z.q_dot_m(i),0];
        w_m(:,i)=subs(w(:,i),vars2replace_motor,vars2replace_motor_) ;
        v_m(:,i)=subs(v(:,i),vars2replace_motor,vars2replace_motor_) ;
        
        vlast_m=v_m(:,i);
        wlast_m=w_m(:,i);
    end
    
    vlast=v(:,i);
    wlast=w(:,i);
    if(movingframes)
        disp("moving frames ONN pay attention!!")
        %         vc(:,i)=collect(simplify(vlast+cross(wlast,rc(:,i))),[dc]);
        vc(:,i)=jacobian(Pc(:,i),q)*qd;
        if(z.ismotor)
            %vc_m(:,i)=jacobian(Pc_m(:,i),z.q_m)*z.q_dot_m;
            vars2replace_motor =[l(i),q(i),qd(i),dc(i)];
            vars2replace_motor_=[l(i),z.q_m(i),z.q_dot_m(i),0];
            vc_m(:,i)=subs(vc(:,i),vars2replace_motor,vars2replace_motor_) ;
        end
        
    else
        disp("moving frames OFF!!")
        vc(:,i)=jacobian(Pc(:,i),q)*qd;
        if(z.ismotor)
%             vc_m(:,i)=jacobian(Pc_m(:,i),z.q_m)*z.q_dot_m;
            vars2replace_motor =[l(i),q(i),qd(i),dc(i)];
            vars2replace_motor_=[l(i),z.q_m(i),z.q_dot_m(i),0];
            vc_m(:,i)=subs(vc(:,i),vars2replace_motor,vars2replace_motor_) ;
        end
    end
    %     vc(:,i)=optiomalSortD(vlast+cross(wlast,rc(:,i)),qall_sincos_comb);
    %     Ic{i}=simplify(RTotali{i}*I{i}*RTotali{i}'); %ya definida
    
    %%konig theorem
    %     Ti2(i)= simplify(0.5*m(i)*collect(simplify(expand(vc3(:,i)'*vc3(:,i))),[qd(1:n)])+0.5*simplify((w(:,i)'*Ic{i}*w(:,i))));%,[qd(1:2)]);
    %     Ti(i)= optiomalSortD(0.5*m(i)*vc(:,i)'*vc(:,i)+0.5*w(:,i)'*Ic{i}*w(:,i),qall_sincos_comb);%,[qd(1:2)]);

    Ti(i)= simplify(0.5*m(i)*collect(simplify(expand(vc(:,i)'*vc(:,i)),'Steps',16),[qd(1:n)])+0.5*simplify((w(:,i)'*Ic{i}*w(:,i)),'Steps',16),'Steps',16)%,[qd(1:2)]);
%     Ti(i)= simplify(0.5*m(i)*collect(simplify(expand(vc(:,i)'*vc(:,i))),[qd(1:n)])+0.5*simplify((w(:,i)'*Ic{i}*w(:,i))))%,[qd(1:2)]); %working
    %Ti(i)= simplify(collect(Ti(i),z.q_dot.^2))%collecting derivative terms
    %this last line was working
    Ti(i)= simplify(collect(Ti(i),z.q_dot.^2),'Steps',16)%collecting derivative terms
    
    if(z.ismotor)
        Ti_m(i)=simplify(0.5*z.m_m(i)*collect(simplify(expand(vc_m(:,i)'*vc_m(:,i))),[z.q_dot_m(1:n)])+0.5*simplify((w_m(:,i)'*z.Ic_m{i}*w_m(:,i))))%,[qd(1:2)]);
        Ti_total(i)=simplify(Ti(i)+Ti_m(i));
    end
end


T=sum(Ti);
M = simplify(getInertiaMatrixFromKE(T,qd));
sincos_ = getShortNotation_SinCosV2(n,2);
if(z.ismotor)
    T_m=simplify(sum(Ti_m));
    T_total=simplify(sum(Ti_total));
    M_m = simplify(getInertiaMatrixFromKE(T_m,z.q_dot_m))
    
    M_total=simplify(getInertiaMatrixFromKE(T_total,[z.q_dot;z.q_dot_m]))
    
    VarShortRobot.M_m= toShortNotation(simplify(M_m),sincos_);
    VarShortRobot.M_total= toShortNotation(simplify(M_total),sincos_);
    VarShortRobot.T_m= toShortNotation(simplify(T_m),sincos_);
    VarShortRobot.Ti_m= toShortNotation(simplify(Ti_m),sincos_);
    VarShortRobot.Ti_total= toShortNotation(simplify(Ti_total),sincos_);
    VarShortRobot.T_total= toShortNotation(simplify(T_total),sincos_);
    VarShortRobot.Pc_m = toShortNotation(simplify(Pc_m),sincos_);
    VarShortRobot.vc_m= toShortNotation(simplify(vc_m),sincos_);
    VarShortRobot.w_m= toShortNotation(simplify(w_m),sincos_);
    
    Trans.M_m= simplify(M_m);
    Trans.M_total= simplify(M_total);
    Trans.T_m= simplify(T_m);
    Trans.Ti_m= simplify(Ti_m);
    Trans.Ti_total= simplify(Ti_total);
    Trans.T_total= simplify(T_total);
    Trans.Pc_m = simplify(Pc_m);
    Trans.vc_m= simplify(vc_m);
    Trans.w_m= simplify(w_m);
end

%  [C,cac,Csubs,S] = getCs(M,q,qd);


    
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
    Trans.Jacobian_PTotal=jacobian(PTotal,z.q);
    local_derivated_jacobian = derivate_D(Trans.Jacobian_PTotal,z.n,z.t,z.vartoderivate);
    Trans.Jacobian_dot_PTotal=local_derivated_jacobian.dot;
%     [converted,new_dep_var,convertedshort] = shortdiff2(diff(Trans.Jacobian_PTotal,z.q),dep_var,inddep_var,on_short,grade,Nvartoreplace)
    Trans.PPartial=PPartial;
    Trans.Pc=Pc;
    Trans.RTotali=RTotali;
    Trans.ThetaPartial=ThetaPartial;
    Trans.ThetaTotal=ThetaTotal;
    Trans.vc= (simplify(vc));
    Trans.w= (simplify(w));
    Trans.T= (simplify(T));
    Trans.Ti= (simplify(Ti));
    Trans.M= (simplify(M));

end

