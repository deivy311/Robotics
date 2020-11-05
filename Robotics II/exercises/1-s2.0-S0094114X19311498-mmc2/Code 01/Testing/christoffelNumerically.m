function [gamma]=getChristoffelSymbolsNumerically_std(T,pcii,Icii,mcii,n)
%% About the function: this is a function that is used to calculate
% Christoffel Symbols for a robotic manipulator Numerically using tensor
% mathematics

%% Areguments:
% T: is (4x4xn) transformation matrix of the serially linked robot, each
% (4x4) matrix represents the transform for each link in the base frame. 
% (T) is defined according to the modefied DH as described by JJ Craig
% pcii: is 3Xn matrix while each column represents the local coordinates
% of the center of mass of each link.
% Icii: is (3x3xn) matrix, each 3x3 matrix of which represnets the
% associated link inertial tensor represented in its local inertial frame
% mcii: is (1xn) vector, each element of which specifies a mass of one of
% the links
% n: is the number of degrees of freedom of the robot

%% Return value:
% gamma: (nxnxn) matrix of Christoffel Symbols
% each element gamma(i,j,k) corresponds to the (i,j,k) Christoffel Symbol

% Copyright Mohammad SAFEEA, 15-Dec-2018

gamma=zeros(n,n,n);
%% Calculate the matrices Li
half_Li=zeros(3,3,n);
for i=1:n
    R=T(1:3,1:3,i);
    Trace=Icii(1,1,i)+Icii(2,2,i)+Icii(3,3,i);
    half_Li(:,:,i)=0.5*R*(Trace*eye(3)-2*Icii(:,:,i))*R';
end

%% Initialization of Mci and Fci.
% (Mci) the inertial moment and (Fci) the inertial force acting on each link due to the
% Coriolis and Centrifugal accelerations
Mci=zeros(3,n,n,n+1);
Fci=zeros(3,n,n,n+1);

temp=zeros(3,1);
for i=1:n % loop over the links
    for j=1:i % loop on the joints (j) that injects Coriolis and Centrifugal accelerations to link (i)
        temp(1)=half_Li(1,1,i)*T(1,3,j)+half_Li(1,2,i)*T(2,3,j)+half_Li(1,3,i)*T(3,3,j);
        temp(2)=half_Li(2,1,i)*T(1,3,j)+half_Li(2,2,i)*T(2,3,j)+half_Li(2,3,i)*T(3,3,j);
        temp(3)=half_Li(3,1,i)*T(1,3,j)+half_Li(3,2,i)*T(2,3,j)+half_Li(3,3,i)*T(3,3,j);
        for m=j:i % perfrom operations on the lower triangular including the diagonal
            % following takes 9 operations
            Mci(1,m,j,i)=temp(2)*T(3,3,m)-temp(3)*T(2,3,m);
            Mci(2,m,j,i)=-temp(1)*T(3,3,m)+temp(3)*T(1,3,m);
            Mci(3,m,j,i)=temp(1)*T(2,3,m)-temp(2)*T(1,3,m);
        end
    end
end

pci=zeros(3,1);
temp_ij=zeros(3,1);
pcim=zeros(3,1);
for i=1:n % loop over the links
    pci(1)=T(1,4,i)+T(1,1,i)*pcii(1,i)+T(1,2,i)*pcii(2,i)+T(1,3,i)*pcii(3,i);
    pci(2)=T(2,4,i)+T(2,1,i)*pcii(1,i)+T(2,2,i)*pcii(2,i)+T(2,3,i)*pcii(3,i);
    pci(3)=T(3,4,i)+T(3,1,i)*pcii(1,i)+T(3,2,i)*pcii(2,i)+T(3,3,i)*pcii(3,i);
    for j=1:i % loop on the joints (j) that injects Coriolis and Centrifugal accelerations to link (i)
        temp_ij(1)=mcii(i)*T(1,3,j);
        temp_ij(2)=mcii(i)*T(2,3,j);
        temp_ij(3)=mcii(i)*T(3,3,j);
        for m=j:i % perfrom operations on the lower triangular including the diagonal
            % following takes 3+2*9=21 operations
            pcim(1)=pci(1)-T(1,4,m);
            pcim(2)=pci(2)-T(2,4,m);
            pcim(3)=pci(3)-T(3,4,m);
            temp(1)=T(2,3,m)*pcim(3)-T(3,3,m)*pcim(2);
            temp(2)=-T(1,3,m)*pcim(3)+T(3,3,m)*pcim(1);
            temp(3)=T(1,3,m)*pcim(2)-T(2,3,m)*pcim(1);
            Fci(1,m,j,i)=temp_ij(2)*temp(3)-temp_ij(3)*temp(2);
            Fci(2,m,j,i)=-temp_ij(1)*temp(3)+temp_ij(3)*temp(1);
            Fci(3,m,j,i)=temp_ij(1)*temp(2)-temp_ij(2)*temp(1);
        end
    end
end


%% Reverse recursion on moments and forces from last link to the first
i=n;
Mac=zeros(3,n,n);
Fac=zeros(3,n,n);
    
for i=n:-1:1
    pcii_a=T(1:3,1:3,i)*pcii(:,i);
    if i==n
        L=zeros(3,1);
    else
        L=T(1:3,4,i+1)-T(1:3,4,i);
    end   
    
    for j=1:n
        for m=j:n
            % following takes 9*3 + 3+ 5 = 35 operations
            Mac(1,m,j)=Mac(1,m,j)+Mci(1,m,j,i)+...
        L(2)*Fac(3,m,j)-L(3)*Fac(2,m,j)+...
        pcii_a(2)*Fci(3,m,j,i)-pcii_a(3)*Fci(2,m,j,i);
    
            Mac(2,m,j)=Mac(2,m,j)+Mci(2,m,j,i)+...
        -L(1)*Fac(3,m,j)+L(3)*Fac(1,m,j)...
        -pcii_a(1)*Fci(3,m,j,i)+pcii_a(3)*Fci(1,m,j,i);


            Mac(3,m,j)=Mac(3,m,j)+Mci(3,m,j,i)+...
        L(1)*Fac(2,m,j)-L(2)*Fac(1,m,j)+...
        pcii_a(1)*Fci(2,m,j,i)-pcii_a(2)*Fci(1,m,j,i);
    
            Fac(1,m,j)=Fac(1,m,j)+Fci(1,m,j,i);
            Fac(2,m,j)=Fac(2,m,j)+Fci(2,m,j,i);
            Fac(3,m,j)=Fac(3,m,j)+Fci(3,m,j,i);
            
            gamma(m,j,i)=T(1,3,i)*Mac(1,m,j)+T(2,3,i)*Mac(2,m,j)+T(3,3,i)*Mac(3,m,j);
            gamma(j,m,i)=gamma(m,j,i);
        end
    end
end
 
end