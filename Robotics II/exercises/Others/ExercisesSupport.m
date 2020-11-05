close all
T = 10;
t = 0:0.005:T;

x = 0.15+0.05*cos(5*pi*t);
y = 0.05*sin(5*pi*t);
plot(x,y)
% 
% h = animatedline('Color','r','Marker','o');
% axis([0.05,0.25,-0.05,0.05])
% 
% grid on
% 
% for k = 1:length(x)
%     addpoints(h,x(k),y(k));
%     drawnow
% end
% 
% datacursormode on

J
pinv