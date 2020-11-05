% Copyright: Mohammad Safeea, July 2019.
% ------------------------------------------------------------
% A script used for Christoffel Symbols calculation for 6DOF robot.
% The calculation is carried out using the proposed recursive algorithm.
% Timing results are reported.

close all;clear all;clc;

n=6; % number of degrees of freedom
iterations=100000;
%% Generating a random pose
disp('Generating a random pose');
q = rand(n,1); % joint angles
disp(q);

%% Christoffel symbols using recursive algorithm
disp('Calculating Christoffel symbols recursively for 6DOF robot');

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

