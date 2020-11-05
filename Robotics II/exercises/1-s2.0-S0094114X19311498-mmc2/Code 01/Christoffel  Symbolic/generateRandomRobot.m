close all;
clear all;
clc;
n=6;
alfa=rand(n,1);
a=rand(n,1);
d=rand(n,1);

% following are inertial data defined in the reference frame of the link, 
pcii=rand(3,n); % position vector of COM of link (i) described in the ith reference frame
mcii=abs(rand(n,1));
Icii=zeros(3,3,n);

for i=1:n
    q=rand(3,1);
    R=rotx(q(1))*roty(q(2))*rotz(q(3));
    R=R(1:3,1:3);
    eig=abs(rand(3,1));
    I=zeros(3,3);
    for j=1:3
        I(j,j)=eig(j);
    end
    Icii(1:3,1:3,i)=R'*I*R;
end

save(['robotStructure_',num2str(n),'DOF.mat'],'n','alfa','a','d','pcii','mcii','Icii')