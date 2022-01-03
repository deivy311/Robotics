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

sigmaD=  upper(sigmaD);

n = size(sigmaD,2); %Number of joints

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

