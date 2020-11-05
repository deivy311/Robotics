%% Copyright: Mohammad Safeea, April-2019
% This is a MATLAB script used to generate the (.m function) of a specific robot.
% The equations in the (.m function) are generated symbolically based on Lagrangian method.

clear all;close all;clc;

dof=5; % define the number of degrees of freedom of the robot
data_fname=['robotStructure_',num2str(dof),'DOF.mat'];
dados=load(data_fname);
n=dados.n;
% assign DH paramenters from data in the (mat) file
alfa=dados.alfa;
a=dados.a;
d=dados.d;

t0=datetime('now')
% following are inertial data defined in the reference frame of the link, 
pcii=dados.pcii; % position vector of COM of link (i) described in the ith reference frame
mcii=dados.mcii;
Icii=dados.Icii;

q = sym('q', [n 1]);
temp=eye(4);
T=[];
for i=1:n
    var=q(i);
    Transform=double(rotx(alfa(i))*trx(a(i))*trz(d(i)))*rotz(var);
    T{i}=(temp*Transform);
    temp=T{i};
end


% calculate the mass matrix M
Jv=sym('Jv',[3,n]);
Jw=sym('Jw',[3,n]);
M=sym('M',[n, n]);
for i=1:n
    for j=1:n
        M(i,j)=0;
    end
end

for i=1:n
    % COM position
    pci=T{i}(1:3,4)+T{i}(1:3,1:3)*pcii(:,i);
    mci=mcii(i);
    for j=1:i
        pj=T{j}(1:3,4);
        kj=simplify (T{j}(1:3,3));
        pcij=simplify(pci-pj);
        Jv(:,j)=(cross(kj,pcij));
        Jw(:,j)=simplify(T{i}(1:3,1:3)'*(kj));
    end
    for j=(i+1):n
        Jv(:,j)=Jv(:,j)*0;
        Jw(:,j)=Jw(:,j)*0;
    end
    
    M=M+...
        (mci*(Jv.'*Jv))+...
        (Jw.'*Icii(:,:,i)*Jw);
    
end


disp('Calculate Christoffel symbols');
chr=sym('chr',[n n n]);
for i=1:n
    for j=1:n
        for k=1:j
            qvar_k=q(k);
            qvar_j=q(j);
            qvar_i=q(i);
            chr(k,j,i) =( 0.5* ( diff(M(i,j), qvar_k) + ...
                diff(M(i,k), qvar_j) - ...
                diff(M(j,k), qvar_i)));
        end
    end
end


disp('Generating Christoffel symbols function')
fname=['chri_symbGen',num2str(dof),'DOF'];


index=0;
for i=1:n
    for j=1:n
        for k=1:j
            index=index+1;
        end
    end
end

chri=sym('chri',[index,1]);
index=0;
for i=1:n
    for j=1:n
        for k=1:j
            index=index+1;
            chri(index)=chr(k,j,i);
        end
    end
end

matlabFunction(chri,'File',fname,'Optimize',true)
t1=datetime('now')

