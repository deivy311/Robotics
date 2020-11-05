% unicycle path planning using flat outputs and path parameterization
% here: cubic polynomials
% G. Oriolo
% DIAG, Sapienza 
% % 

clear all;close all;

% % 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% to be chosen by the user %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 
% forward parking
x_i=0; y_i=0; theta_i=0;     % initial configuration
x_f=4; y_f=-2; theta_f=-pi/4;  % final configuration
% % 
% parallel parking
%x_i=0; y_i=0; theta_i=0;     % initial configuration
%x_f=0; y_f=4; theta_f=0;  % final configuration
% % 
% pure reorientation
%x_i=0; y_i=0; theta_i=0;     % initial configuration
%x_f=0; y_f=0; theta_f=pi/2;  % final configuration
% % 
N=100; % number of sampling intervals in s=[0,1]
% % 
% forward or backward motion (k>0 or <0)
k=-10%5;   % k=ki=kf same value for initial and final geometric velocity 

%%%%%%%%%%%%%%%
% computation %
%%%%%%%%%%%%%%%

% parameters of the cubic polynomial
% % 
alfa=[k*cos(theta_f)-3*x_f; k*sin(theta_f)-3*y_f];
alfa_x=alfa(1);
alfa_y=alfa(2);
% % 
beta=[k*cos(theta_i)+3*x_i;k*sin(theta_i)+3*y_i];
beta_x=beta(1);
beta_y=beta(2);
% % 
% vector for trajectory parametrization
% % 
s=(1/N)*[0:1:N]; 

% these are the interpolating polynomials
% x(s),y(s) and their first and second derivative wrt s
% % 
x=-(s-1).^3*x_i+s.^3*x_f+alfa_x*(s.^2).*(s-1)+beta_x*s.*((s-1).^2);
y=-(s-1).^3*y_i+s.^3*y_f+alfa_y*(s.^2).*(s-1)+beta_y*s.*((s-1).^2);
% % 
xp= -3*(s-1).^2 *x_i+3*s.^2 *x_f+alfa_x*(3*s.^2-2*s)+beta_x*(3*s.^2-4*s +1); 
yp= -3*(s-1).^2 *y_i+3*s.^2 *y_f+alfa_y*(3*s.^2-2*s)+beta_y*(3*s.^2-4*s +1); 
% % 
xpp= -6*(s-1)*x_i+6*s*x_f+alfa_x*(6*s-2)+beta_x*(6*s-4); 
ypp= -6*(s-1)*y_i+6*s*y_f+alfa_y*(6*s-2)+beta_y*(6*s-4); 
 
% theta(s) and geometric inputs are reconstructed via flatness formulas
% % 
if sign(k)==1                         % fwd motion
        theta=atan2(yp,xp);
else                                 % bwd motion
        theta=atan2(yp,xp)-pi;
end
% % 
tildev=xp.*cos(theta)+yp.*sin(theta);
tildeomega=(ypp.*xp-xpp.*yp)./(xp.^2+yp.^2);


%======================================================================

% plotting of results
% % 
hold off
clf
% % 
figure(1);
%subplot(2,2,1)
hold on;
axis equal;
set(gca,'fontname','Times','fontsize',12,'fontweight','normal');box on;
% % 
%setup unicycle shape
% % 
unicycle_size=0.3;
vertices_unicycle_shape=unicycle_size*[[-0.25;-0.5;1/unicycle_size],...
    [0.7;0;1/unicycle_size],[-0.25;0.5;1/unicycle_size]];
faces_unicycle_shape=[1 2 3];
% % 
%plot unicycle initial configuration
% % 
M=[cos(theta_i) -sin(theta_i) x_i; sin(theta_i) cos(theta_i)  y_i;0 0 1]; 
vertices_unicycle_shape_i=(M*vertices_unicycle_shape)';
vertices_unicycle_shape_i=vertices_unicycle_shape_i(:,1:2);
% % 
patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_i,...
    'FaceColor','none','EdgeColor','r');
% % 
%plot unicycle final configuration
% % 
M=[cos(theta_f) -sin(theta_f) x_f; sin(theta_f) cos(theta_f)  y_f;0 0 1];
vertices_unicycle_shape_f=(M*vertices_unicycle_shape)';
vertices_unicycle_shape_f=vertices_unicycle_shape_f(:,1:2);
patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_f,'FaceColor','none','EdgeColor','b');
% % 
%plot trajectory
% % 
plot(x,y,'k')
xlabel('[m]');ylabel('[m]');
% % 
range=axis;
incr=0.03;
range(1)=range(1)-(range(2)-range(1))*incr;
range(2)=range(2)+(range(2)-range(1))*incr;
range(3)=range(3)-(range(4)-range(3))*incr;
range(4)=range(4)+(range(4)-range(3))*incr;
axis(range);
axis equal
range=axis; 
% % 
%plot theta
figure(2);
set(gca,'fontname','Times','fontsize',12,'fontweight','normal');
plot(s,theta,'k');
ylabel('[rad]');
title('theta(s)')
box on;
% % 
%plot geometric inputs
% % 
figure(3);
subplot(2,1,1); hold on;
set(gca,'fontname','Times','fontsize',12,'fontweight','normal');
plot(s,tildev,'k');
ylabel('[m]');
title('tilde v(s)')
box on;
% % 
subplot(2,1,2); hold on;
set(gca,'fontname','Times','fontsize',12,'fontweight','normal');
plot(s,tildeomega,'k')
ylabel('[rad]');
title('tilde omega(s)')
box on;

