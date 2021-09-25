function [f,J,V,J_ps,PCVar,PC_EEShort,VarShortRobot,PC_EE] = getGenericPC_EE2(z)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Gets a prebuild model of the RR Robot
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
xyonly=z.xyonly;
task_defined=z.task_defined;
sigmaD=  upper(sigmaD);
sigmaD(find(sigmaD=='P'))=1 ;
sigmaD(find(sigmaD=='R'))=0 ;
% sigma = [0;0] % 0: Revolute, 1: Prismatic
n = size(sigmaD,2); %Number of joints
qallcomb=[];

z.sigmaD=sigmaD;
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
sincos_ = getShortNotation_SinCosV2(n,2);

% [Pc,vc,w,T,Ti,M,VarShortRobot,Trans] = getGenericPC(sigmaD,l,q,qd,dc,m,I);
[Pc,vc,w,T,Ti,M,VarShortRobot,Trans] = getGenericPC2(z);

% qtotal=sum(q'.*(1-sigmaD(1,:)));
% f=Pc(:,end);

f=Trans.PTotal(:,end);
PCVar.PTotali=Trans.PTotali(1:2,:);

if(xyonly)
    f=f(1:2,:);
    
else
    if(~isempty(task_defined))
        [~,num_new_tasks]=size(task_defined)
        f=f(1:2,:);%% check it
        f_=[];
        redundancy_tasks.J_k=cell(1,num_new_tasks);
        redundancy_tasks.f_k=cell(1,num_new_tasks);
        for i_task=1:num_new_tasks
            task_temp=task_defined{i_task};
            %             [local_size_task,~]=size(task_temp.f);
            if(task_temp.f=='end_effector')
                task_temp.f=f;
            end
            if(ismember('pd_',fieldnames(task_temp)))
                redundancy_tasks.pd_{i_task}=task_temp.pd_;
                
            end
            
            if(ismember('pd_dot_',fieldnames(task_temp)))
                redundancy_tasks.pd_dot_{i_task}=task_temp.pd_dot_;
            end
            
            f_=[f_;task_temp.f];
            redundancy_tasks.f_k{i_task}=task_temp.f;
            redundancy_tasks.J_k{i_task}=jacobian(task_temp.f,q);
            %             PCVar.PTotali=[PCVar.PTotali;task_temp.Pc]; %% not uncomment
            %             this line , because it was already computed
        end
        f=f_;
    else
        f(end,:)=Trans.ThetaTotal;
        
    end
    PCVar.PTotali(end+1,:)=cumsum(Trans.ThetaPartial)';
end
J=jacobian(f,q);  
V=optiomalSortD(J*qd,qall_sincos_comb);
J_ps=simplify(pinv(J));
PCVar.Pc=Pc;
PCVar.vc=vc;

PCVar.f_derivated=derivate_D(f,n,z.t,"q");
PCVar.f_dot=PCVar.f_derivated.dot;
PCVar.f_ddot=PCVar.f_derivated.ddot;
PCVar.f_tdot=PCVar.f_derivated.tdot;

PCVar.pc_derivated=derivate_D(Pc,n,z.t,"q");
PCVar.Pc_ddot=PCVar.pc_derivated.ddot;
PCVar.w=w;
PCVar.T=T;
PCVar.Ti=Ti;
PCVar.M=M;

% PC_EE.redundancy_tasks=redundancy_tasks;
if(~isempty(task_defined))
    PC_EE=redundancy_tasks;
end
PC_EE.f=f;
PC_EE.J=J;
PC_EE.J_derivated=derivate_D(J,n,z.t,"q");
PC_EE.J_dot=PC_EE.J_derivated.dot;
PC_EE.J_tdot=PC_EE.J_derivated.tdot;
PC_EE.J_ps=J_ps;
PC_EE.V=V;
PC_EEShort.f=toShortNotation(f,sincos_);
PC_EEShort.J=toShortNotation(J,sincos_);
%adding derivative terms
PC_EEShort.J_dot_short=toShortNotation(PC_EE.J_dot,sincos_);
PC_EEShort.V=toShortNotation(V,sincos_);
PC_EEShort.Pc=Pc;
PC_EEShort.Pc_ddot=PCVar.pc_derivated.ddot_short;
PC_EEShort.PTotali=Trans.PTotali;
end

