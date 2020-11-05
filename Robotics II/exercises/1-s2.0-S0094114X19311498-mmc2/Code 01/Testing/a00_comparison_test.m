% Copyright: Mohammad Safeea, April 2019.
% ------------------------------------------------------------
% A script used for Christoffel Symbols calculation comparison.
% The calculation is carried out using two different methods:
% 1- The proposed Recursive method implemented in the function
% (christoffelNumerically.m). 
% 2- An offline (Symbolically) generated function
% (chri_symbGen5DOF.m) for the optimized symbolic
% equations
% (chri_symbGen5DOFnoOpt.m) for the 
% symbolic equations without any optemization

% Comparison is in terms of:
% 1- Execution time
% 2- Error in numerical results

close all;clear all;clc;

n=5;
iterations=100000;
%% Generating a random pose
disp('Generating a random pose');
q = rand(n,1); % joint angles
disp(q);
%% Christoffel symbols using symbolic function (symbolic equations optimized)
disp('Calculating Christoffel symbols from symbolic function (with symbolic equations optimization) ');
tic;
chr_symbolic=zeros(n,n,n);
for temp=1:iterations
    y = chri_symbGen5DOF(q(1),q(2),q(3),q(4),q(5));
    index=0;
    for i=1:n
        for j=1:n
            for k=1:j
                index=index+1;
                chr_symbolic(k,j,i)=y(index);
                chr_symbolic(j,k,i)=y(index);
            end
        end
    end
end
t=toc;
disp('Using Symbolic function (optimized), required time for calculation is');
disp(t/iterations);
%% Christoffel symbols using recursive algorithm
disp('Calculating Christoffel symbols recursively');

dados=load(['robotStructure_',num2str(n),'DOF.mat']);
a=dados.a;
alfa=dados.alfa;
d=dados.d;
pcii=dados.pcii;
Icii=dados.Icii;
mcii=dados.mcii;

T=GetKenimaticModelAccelerated(a,alfa,q,d);
tic;
for temp=1:iterations
    [gamma]=christoffelNumerically(T,pcii,Icii,mcii,n);
end
t=toc;
disp('Using recursive algorithm, required time for calculation is');
disp(t/iterations);

%% Christoffel symbols using symbolic function (symbolic equations are not optimized)
disp('Calculating Christoffel symbols from symbolic function (without symbolic equations optimization) ');
tic;
chr_symbolic_slow=zeros(n,n,n);
y = chri_symbGen5DOFnoOpt(q(1),q(2),q(3),q(4),q(5));
index=0;
for i=1:n
    for j=1:n
        for k=1:j
            index=index+1;
            chr_symbolic_slow(k,j,i)=y(index);
            chr_symbolic_slow(j,k,i)=y(index);
        end
    end
end
t=toc;
disp('Using Symbolic function (not optimized), required time for calculation is');
disp(t);

%% Error in calculations:
c1=gamma;
c2=chr_symbolic;
sum=0;
for i=1:n
    for j=1:n
        for k=1:n
            if abs(c1(i,j,k))<1e-14
%                 fprintf('The element (%d,%d,%d) of Christoffel symbols is close to zero, less than (e-14) \n',i,j,k);
%                 disp('As such, it was not taken into consideration in error calculation');
            else
                sum=sum+abs(c1(i,j,k)-c2(i,j,k))/abs(c1(i,j,k)+c2(i,j,k));
            end
        end
    end
end
error=2*sum/(n*n*n);
disp('The relative error in the calculations between the different methods is:')
disp(error);
