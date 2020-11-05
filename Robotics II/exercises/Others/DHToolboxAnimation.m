%Simple script to simulate a robot gicen its Denavit-Hartenberg parameters
%Must be defined in the order  theta | d | a | alpha

%http://www.daslhub.org/unlv/wiki/doku.php?id=robotic_manipulators
close all; clear;
dh = [0,0,1,0;
      0,0,1,0;
      0,0,1,0]

%r = SerialLink(dh)
r = SerialLink([0 0 10 0;
                0 0 10 0;
                0 0 10 pi/2], 'name', 'three link')
r.teach
%%
L(1) = Revolute('d', 0, 'a', 20, 'alpha', pi/2);
L(2) = Prismatic('alpha', 0);
twolink = SerialLink(L, 'name', 'two link')
twolink.teach
%% SCARA ROBOT
clear
links = [
    Revolute('d', 0.387, 'a', 0.325, 'qlim', [-50 50]*pi/180);
    Revolute('a', 0.275, 'alpha', pi, 'qlim', [-88 88]*pi/180);
    Prismatic('qlim', [0 0.210]);
    Revolute()
    ];

c600 = SerialLink(links, 'name', 'Cobra600', 'manufacturer', 'Adept', ...
    'plotopt', {'workspace', [0 0.8 -0.6 0.6 0 0.4]} );
c600.teach
%%
clear; close all

mdl_puma560

qr=[0 1 0 1 0 1]
T_1=p560.fkine(qr)
p560.teach(qr)

qr_2=p560.ikine6s(T_1)
T_2=p560.fkine(qr_2)


%%
mdl_stanford
% clear; %For some reason, I can't make it work every time without using
% the prebuild model.
% 
% L(1) = Link([ 0     0.412   0   -pi/2     0]);
% L(2) = Link([ 0     0.154   0    pi/2     0]);
% L(3) = Link([ -pi/2 0       0    0        1]);  % PRISMATIC link
% L(4) = Link([ 0     0       0   -pi/2     0]);
% L(5) = Link([ 0     0       0    pi/2     0]);
% L(6) = Link([ 0     0.263   0    0        0]);
% stanf = SerialLink(L, 'name', 'Stanford arm');
% stanf.teach

% qz = [0 0 0 0 0 0];
% 
% stanf = SerialLink(L, 'name', 'Stanford arm');
% stanf.plotopt = {'workspace', [-2 2 -2 2 -2 2]};

stanf.teach
%%
%SCARA Robot
close all
mdl_cobra600
c600.plot([0 0 0 0],'workspace', [-0.5 1 -0.5 0.5 -0.5 1])
c600.teach

%%
%Creating the DASL serial arm on the RVC toolbox.
%L(n)=LINK([theta D A alpha Sigma(optional) Offset(optional)])

L1=11;
L2=15;
L3=10;
L4=21;
L5=8;
pi=3.1416;

q0=[0 0 0 0 0];

L(1)=Link([0 -L1 0 -pi/2 0]);
L(2)=Link([0 0 L2 0 0]);
L(3)=Link([0 0 L3 0 0]);
L(4)=Link([0 0 0 pi/2 0]);
L(5)=Link([0 -(L4+L5) 0 0 0]);


DaslArm=SerialLink(L,'name','DaslArm');

DaslArm.teach(q0)

%%
%END EFFECTOR POSITION
% trotx(angle) = rotate (angle) about the x axis 
% troty(angle) = rotate (angle) about the y axis
% trotz(angle) = rotate (angle) about the z axis
% transl(x,y,z) = pure translation of x , y , z
% The end effector pose will be a combination of tranls and trot

%For example lets use a pure translation .
T=transl(20,35,-30);

%IK SOLVER
%Initial pose (all rotations = 0)
q0=[0 0 0 0 0];
%Mask to be used on the IK solver (x y z rotx roty rotz)
m=[1 1 1 0 0 0];

%Solving the IK and converting to degrees
q_ikine=DaslArm.ikine(T,q0,m);
q_degrees=q_ikine*(180/pi);

%Creating a trajectory from q0 to the IK solution
%Time variable
t=[0:0.05:4];
%trajectory
q_TRAJ=jtraj(q0,q_ikine,t);

%Plot the result
DaslArm.plot(q_TRAJ)

%%
clear;
mdl_puma560
T = p560.fkine(qn)
p560.teach(qr) %other positions like qz, qn or qs, or custom, can be applied


%%
clear
mdl_baxter
left
% left = 
%  
% Baxter LEFT [Rethink Robotics]:: 7 axis, RRRRRRR, stdDH, slowRNE 
% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|       0.27|      0.069|    -1.5708|          0|
% |  2|         q2|          0|          0|     1.5708|     1.5708|
% |  3|         q3|      0.364|      0.069|    -1.5708|          0|
% |  4|         q4|          0|          0|     1.5708|          0|
% |  5|         q5|      0.374|       0.01|    -1.5708|          0|
% |  6|         q6|          0|          0|     1.5708|          0|
% |  7|         q7|       0.28|          0|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+
% base:    t = (0.0646, 0.259, 0.119), RPY/xyz = (0, 0, 45) deg  
TE = SE3(0.8, 0.2, -0.2) * SE3.Ry(pi);
q = left.ikine(TE)
left.fkine(q).print('xyz')

left.plot(q)

%% Check singularity points
clear
t =0:0.1:2
mdl_puma560
T1 = SE3(0.5, 0.3, 0.44) * SE3.Ry(pi/2);
T2 = SE3(0.5, -0.3, 0.44) * SE3.Ry(pi/2);

Ts = ctraj(T1, T2, length(t));

qc = p560.ikine6s(Ts)
m = p560.maniplty(qc);
plot(qc)
%% Writing on a surface
clear; close all
mdl_puma560

load hershey %It's a font
B = hershey{'B'}
B.stroke %Points to be drawn. Nan = pen lifted

path = [ 0.25*B.stroke; zeros(1,numcols(B.stroke))];
k = find(isnan(path(1,:)));
path(:,k) = path(:,k-1); path(3,k) = 0.2;

traj = mstraj(path(:,2:end)', [0.5 0.5 0.5], [], path(:,1)',0.02, 0.2);

about(traj) %Steps in interpolated path
numrows(traj)

plot3(traj(:,1), traj(:,2), traj(:,3)) %20ms/sample interval

Tp = SE3(0.6, 0, 0) * SE3(traj) * SE3.oa( [0 1 0], [0 0 -1]);
figure
q = p560.ikine6s(Tp); %Inverse kinematics
p560.plot(q)

%% Manipulability
clear ; close all
mdl_planar2
q = [30 40]
p2.vellipse(q, 'deg')

p2.teach(q,'callback', @(r,q) r.vellipse(q), 'view', 'top','workspace',[-3,3,-3,3,-2,1])
%axis([-3 3 -3 3 -2 1])

%% Panda Robot
%mdl_panda
deg = pi/180;
mm = 1e-3;
d = [333 0 316 0 384 0 0]'*mm;

a = [0 0 0 82.5 -82.5 0 88]'*mm;

alpha = [0 -pi/2 pi/2 pi/2 -pi/2 pi/2 pi/2]';

theta = zeros(7,1);

DH = [theta d a alpha];

% and build a serial link manipulator

% offsets from the table on page 4, "Mico" angles are the passed joint
% angles.  "DH Algo" are the result after adding the joint angle offset.
qz = [0 0 0 0 0 0 0];
qr = [0 -90 -90 90 0 -90 90]*deg;
panda = SerialLink(DH, ...
    'configs', {'qz', qz, 'qr', qr}, ...
    'name', 'PANDA', 'manufacturer', 'Franka-Emika');
panda.teach
%%

