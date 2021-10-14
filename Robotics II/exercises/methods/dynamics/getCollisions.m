%aurhor David Esteban Imbajoa Ruiz
function [Pc,vc,w,T,Ti,M,VarShortRobot,Trans,zout] = getCollisions(z)
%sigmaD,l,q,qd,dc,m,I,methodD,dhTable,firstParam,rc,movingframes,angle_)


sigmaD=z.sigmaD;
l=z.l;
q=z.q;
vars2replace=[];
vars2replace_=[];
if(isempty(z.l_))
    l_=z.l;
    % vars2replace=[vars2replace]
else
    l_=z.l_;
    vars2replace=[vars2replace;l];
    vars2replace_=[vars2replace_;l_];
end
if(isempty(z.q_))
    q_=z.q;
    % vars2replace=[vars2replace]
else
    q_=z.q_;
    vars2replace=[vars2replace;q];
    vars2replace_=[vars2replace_;q_];
end
if(isempty(z.q_dot_))
    q_=z.q_dot;
    % vars2replace=[vars2replace]
else
    q_dot_=z.q_dot_;
    vars2replace=[vars2replace;z.q_dot];
    vars2replace_=[vars2replace_;q_dot_];
end
% q_dot=z.q_dot;
% dc=z.dc;
% m=z.m;
% I=z.I;
% methodD=z.methodD;
% dhTable=z.dhTable;
% firstParam=z.firstParam;
% rc=z.rc;
% movingframes=z.movingframes;
% angle_=z.angle_;
% opt_expr=z.opt_expr;
% collides=z.collides
% prismatic_CoM_method=z.prismatic_CoM_method;
% %UNTITLED Summary of this function goes here
% %   Detailed explanation goes here
% % Gets a prebuild model of the RR Robot
% if(~exist('movingframes'))
%     movingframes=false;
%     
% end
% if(~exist('methodD'))
%     methodD=1;
% end
% if(~exist('firstParam'))
%     firstParam='alpha';
% end
sigmaD=  upper(sigmaD);
% sigmaD(find(sigmaD=='P'))=1 ;
% sigmaD(find(sigmaD=='R'))=0 ;
% % sigma = [0;0] % 0: Revolute, 1: Prismatic
n = size(sigmaD,2); %Number of joints
% qallcomb=[];
% 
% if(~exist("angle_"))
%     angle_=sym(zeros(1,n))
%     %     switch sigmaD(2,i)
%     %     case 1*'X'
%     %         angle_=0
%     %     case 1*'Y'
%     %         angle_=pi/2
%     %     case 1*'Z'
%     %         %angle_=0
%     %          ME = MException('case in Z axis not programmed!!1');
%     %          throw(ME)
%     %     otherwise
%     %angle_=sym('alpha','real')
%     %     end
% end
% for ki=1:n%n for organizee similar parameters
%     qallcomb=[qallcomb,sum(nchoosek(q,ki),2)'];
%     
% end
% qall_sincos_comb=[sin(qallcomb),cos(qallcomb)];
% %
% % q = sym('q', [n 1],'real');
% % d = sym('d', [n 1],'real');
% % a = sym('a', [n 1],'real');
% % qd = sym('qd',[n 1],'real')
% % l = sym('l',[n 1],'real');
% % d = sym('d',[n 1],'real');
% % m = sym('m',[n 1],'real');
% % syms g0
% % g = [0 -g0 0].';
% %
% % % alpha,a, d, theta
% % dhTable = [0 l(1) 0 q(1);
% %            0 l(2) 0 q(2)];
% %
% % % Initialization
% % I = sym(zeros(3,3,n));
% % rcdefined=z.rcdefined;
% % if(~exist('rc'))
% %     rcdefined=true;
% %     rc = [];
% % end
% if(isempty(rc))
%     rcdefined=true
%     z.rcdefined=true
% end
% if(~exist('I'))
%     I={};
% elseif(isempty(I))
%     I= cell(1,n);
%     
% elseif(~iscell(I))
%     I= sym2cell(I);
%     
% end
% disp("si falla aquí está el error")
% charaxis=["x","y","z"];
% 
% %
% % [KE, T] = getKEwithJacobian(dhTable,sigma,q,qd,l,m,I,d,0);
% % M = getInertiaMatrixFromKE(KE,qd);
% % g_q = getGravityWithJacobian(dhTable,q,l,m,d,g,0)
% % % M = getM(dhTable, sigma, qd, rc, m, I, 0);
% % % M = getM(dhTable, sigma,q, qd, l, m, I,d, 0);%implementing doens't
% % % working
% % cS = getCs(M,q,qd);
% % % gravity = Gravity(dhTable,g ,m,rc,q);%% i don't know
% %
% % [Msubs, dynParams, a]= getDynamicParameters(M, q, []);
% if(methodD==1)
%     % [TTotal TPartial,RTotal,RPartial,PTotal, PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]= getTransformationMatrixConfig(sigmaD,l,q,dc,false);
%     %prismatic_CoM_method=1 dc base until center change to value 2 if it
%     % is in the another way
% %     [TTotal TPartial,RTotal,RPartial,PTotal,
% %     PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]=
% %     getTransformationMatrixConfigbyangle(sigmaD,l,q,dc,false,angle_,prismatic_CoM_method);%uncomment
% %     this and comment the next if you have problems
%     [TTotal TPartial,RTotal,RPartial,PTotal, PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]= getTransformationMatrixConfigbyangle(sigmaD,l,q,dc,false,angle_,prismatic_CoM_method,z.xyx_offset,z);
%     
%     disp("method with PC , pay attention!")
% else
%     %%
%     disp("method with DH table, pay attention!")
%     [TTotal TPartial,RTotal,RPartial,PTotal, PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]= getTransformationMatrixConfigfromDH(dhTable,l,q,dc,false,firstParam);
%     
%     ThetaPartial=[];
%     ThetaTotal=[];
% end
% if(~isempty(opt_expr))
%     TTotal =subs(TTotal,opt_expr{1},opt_expr{2})
%     RTotal=subs(RTotal,opt_expr{1},opt_expr{2})
%     PTotal=subs(PTotal,opt_expr{1},opt_expr{2})
%     Pc=subs(Pc,opt_expr{1},opt_expr{2})
%     ThetaTotal=subs(ThetaTotal,opt_expr{1},opt_expr{2})
%     ThetaPartial=subs(ThetaPartial,opt_expr{1},opt_expr{2})
%     PTotali=subs(PTotali,opt_expr{1},opt_expr{2})
%     
%     for ird=1n
%         TPartial{ird}=subs(TPartial{ird},opt_expr{1},opt_expr{2})
%         RPartial{ird}=subs(RPartial{ird},opt_expr{1},opt_expr{2})
%         PPartial{ird}=subs(PPartial{ird},opt_expr{1},opt_expr{2})
%         RTotali{ird}=subs(RTotali{ird},opt_expr{1},opt_expr{2})
%         
%         
%     end
% end
% for i = 1:n
%     %     if(rcdefined||isempty(rc))
%     if(rcdefined)
%         rc = [rc [-l(i)+dc(i); 0; 0]];%%creoq eu cambia respecto al axis
%     end
%     I2(:,:,i) = diag(sym(strcat({'Ixx','Iyy','Izz'},int2str(i)),'real'));%no funciona
%     if(isempty(I{i}))
%         
%         %         I{i}=sym("I"+i+repmat(charaxis,3,1)'+repmat(charaxis,3,1),'real');
%         %         Ic{i}=simplify(RTotali{i}*I{i}*RTotali{i}');
%         Ic{i}=I2(:,:,i);
%     else
%         %         Ic{i}=simplify(RTotali{i}*I{i}*RTotali{i}');%% if it is alrady IC comment and decoment the line below
%         Ic{i}=simplify(I{i});
%         
%     end
% end
% w0=[0,0,0]';
% v0=[0,0,0]';
% wlast=w0;
% vlast=v0;
% Zi=[0,0,1]';
% Zi_=[0,0,1]';
% 
% %%modification added by David
% 
% % [KE, T] = getKEwithJacobianTrans(TPartial, sigmaD(1,:)*1, q, qd, l, m, I, dc,0)%%no funciona creo
% % Pc(end,end)=ThetaTotal;
% for i=1:n
%     %     Zi=RPartial{i}(:,end);%si falla edte es el error
%     if(z.global_q_reference)%% test it !!!!
%         %%using global reference for q!! pay attention
%         w_last_i=   [wlast+(1-sigmaD(1,i))*z.q_dot_global(i)*Zi];
%     else
%         w_last_i=   [wlast+(1-sigmaD(1,i))*q_dot(i)*Zi];
%     end
%     w(:,i)=simplify(RPartial{i}'*w_last_i);
%     v(:,i)=simplify(RPartial{i}'*[vlast+(sigmaD(1,i)*1)*q_dot(i)*(Zi)+cross(w_last_i,PPartial{i})]);
%     
%     vlast=v(:,i);
%     wlast=w(:,i);
%     if(movingframes)
%         disp("moving frames ONN pay attention!!")
%         %         vc(:,i)=collect(simplify(vlast+cross(wlast,rc(:,i))),[dc]);
%         vc(:,i)=jacobian(Pc(:,i),q)*q_dot;
%         
%         
%     else
%         disp("moving frames OFF!!")
%         vc(:,i)=jacobian(Pc(:,i),q)*q_dot;
%         
%     end
%     %     vc(:,i)=optiomalSortD(vlast+cross(wlast,rc(:,i)),qall_sincos_comb);
%     %     Ic{i}=simplify(RTotali{i}*I{i}*RTotali{i}'); %ya definida
%     
%     %%konig theorem
%     %     Ti2(i)= simplify(0.5*m(i)*collect(simplify(expand(vc3(:,i)'*vc3(:,i))),[qd(1:n)])+0.5*simplify((w(:,i)'*Ic{i}*w(:,i))));%,[qd(1:2)]);
%     %     Ti(i)= optiomalSortD(0.5*m(i)*vc(:,i)'*vc(:,i)+0.5*w(:,i)'*Ic{i}*w(:,i),qall_sincos_comb);%,[qd(1:2)]);
%     Ti(i)= simplify(0.5*m(i)*collect(simplify(expand(vc(:,i)'*vc(:,i))),[q_dot(1:n)])+0.5*simplify((w(:,i)'*Ic{i}*w(:,i))));%,[qd(1:2)]);
%     
% end
% 
% T=sum(Ti);
% M = simplify(getInertiaMatrixFromKE(T,q_dot));
% %  [C,cac,Csubs,S] = getCs(M,q,qd);
% 
% 
% sincos_ = getShortNotation_SinCosV2(n);
% % % Jl_ = toShortNotation(Jl,sincos_)
% VarShortRobot.Pc = toShortNotation(simplify(Pc),sincos_);
% VarShortRobot.vc= toShortNotation(simplify(vc),sincos_);
% VarShortRobot.w= toShortNotation(simplify(w),sincos_);
% VarShortRobot.T= toShortNotation(simplify(T),sincos_);
% VarShortRobot.Ti= toShortNotation(simplify(Ti),sincos_);
% VarShortRobot.M= toShortNotation(simplify(M),sincos_);
% 
% 
% Trans.TTotal=TTotal;
% Trans.PTotali=PTotali;
% Trans.TPartial=TPartial;
% Trans.RTotal=RTotal;
% Trans.RPartial=RPartial;
% Trans.PTotal=PTotal;
% Trans.PPartial=PPartial;
% Trans.Pc=Pc;
% Trans.RTotali=RTotali;
% Trans.ThetaPartial=ThetaPartial;
% Trans.ThetaTotal=ThetaTotal;
[Pc,vc,w,T,Ti,M,VarShortRobot,Trans] = getGenericPC2(z);
%% collisions computation
if(~isempty(z.collides))
    zout.J_c_collisions_={};
    num_coll=length(z.collides);
    Pc_collisions=[];%sym('Pc_colls_',[num_coll,1],'real')
    vc_collisions=[];
    J_c_collisions={};
    Vc_T_dot_Fk=[];
    Tau_c=[];
    for icollides=1:num_coll
        l_collide=z.collides(icollides);
        Pc_collisions=[Pc_collisions,subs(Pc(:,l_collide.joint),z.dc(l_collide.joint),l_collide.F_pos)];
        J_c_collisions{icollides}=jacobian(Pc_collisions(:,icollides),q);
        %F
        %joint
        %F_pos
        %             w_last_i=   [wlast+(1-sigmaD(1,l_collide.joint))*qd(i)*Zi];
        %             w(:,i)=simplify(RPartial{i}'*w_last_i);
        %             v(:,i)=simplify(RPartial{i}'*[vlast+(sigmaD(1,i)*1)*qd(i)*(Zi)+cross(w_last_i,PPartial{i})]);
        
        %             vlast=v(:,i);
        %             wlast=w(:,i);
        if(z.movingframes)
            disp("moving frames ONN pay attention!!")
            %                 vc_collisions=[vc_collisions,subs(vc(:,icollides),z.dc(icollides),l_collide.F_pos)];
            
            vc_collisions=[vc_collisions,J_c_collisions{icollides}*z.q_dot];
            
            
        else
            disp("moving frames OFF!!")
            vc_collisions=[vc_collisions,J_c_collisions{icollides}*z.q_dot];
            
        end
        Vc_T_dot_Fk=[Vc_T_dot_Fk;simplify(vc_collisions(:,icollides)'*l_collide.F)]
        Tau_c=[Tau_c,J_c_collisions{icollides}'*l_collide.F]
        %             Ti(i)= simplify(0.5*m(i)*collect(simplify(expand(vc(:,i)'*vc(:,i))),[qd(1:n)])+0.5*simplify((w(:,i)'*Ic{i}*w(:,i))));%,[qd(1:2)]);
        if(~isempty(vars2replace))
            zout.J_c_collisions_{icollides}=subs(J_c_collisions{icollides},vars2replace,vars2replace_)
        end
    end
    
end
%% computing gravity %% comment this if appears some error and do it manually
% isMotor=false
[zout.C,zout.cac,zout.Csubs,zout.S] = getCs(M,z.q,z.q_dot);
[zout.g_q, zout.PE, zout.U,zout.g_q_short,zout.PE_short] = getGravityWithPc(Pc, z.q, z.l, z.m, z.dc, z.g, z.isMotor)

zout.Pc_collisions=Pc_collisions;
zout.vc_collisions=vc_collisions;
zout.J_c_collisions=J_c_collisions;
zout.Vc_T_dot_Fk=Vc_T_dot_Fk;
zout.Tau_c=simplify(Tau_c);

zout.p_r=M*z.q_dot;%momentum
zout.tau=sym('tau_',[n,1],'real');
zout.r=sym('r_',[n,1],'real');
zout.P_0=sym('P_0_',[n,1],'real');
zout.to_integrate=zout.tau+zout.S'*z.q_dot-zout.g_q+zout.r;

if(~isempty(vars2replace))
    zout.Pc_collisions_=subs(zout.Pc_collisions,vars2replace,vars2replace_);
    zout.vc_collisions_=subs(zout.vc_collisions,vars2replace,vars2replace_);
    zout.Vc_T_dot_Fk_=subs(zout.Vc_T_dot_Fk,vars2replace,vars2replace_);
    zout.Tau_c_=subs(zout.Tau_c,vars2replace,vars2replace_);
end

end

