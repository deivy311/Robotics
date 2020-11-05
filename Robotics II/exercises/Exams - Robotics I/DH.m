close all
clear all
clc

NJoints=4

%q = sym('q',[1 4])
syms alpha a d theta q [1 4]
syms alphai ai di thetai qi
% d_s = sym('d',[1 4])
dh = [pi/2      0       202 q(1);
        0       210.5   0   q(2);
        0       268     0   q(3);
        0    174.5   0   q(4)]
dh=[alpha(:) a(:) d(:) theta(:)]

% dh(alpha)= [alpha(1)  a(1)    d(1)    q(1);
%             alpha(2)  a(2)    d(2)    q(2);
%             alpha(3)  a(3)    d(3)    q(3);
%             alpha(4)  a(4)    d(4)    q(4)]
DHParameterName="alpha" 
localvalues=[pi/2      0        202 q(1);
             0       210.5      0   q(2);
             0       268        0   q(3);
             0       174.5      0   q(4)]
Aiqi=[cos(thetai) -cos(alphai)*sin(thetai)  sin(alphai)*sin(thetai) ai*cos(thetai);
      sin(thetai)  cos(alphai)*cos(thetai) -sin(alphai)*cos(thetai) ai*sin(thetai);
      0            sin(alphai)              cos(alphai)             di
      0            0                        0                       1]
for ik=1:NJoints
 eval("alpha"+num2str(ik)+"="+string(localvalues(ik,1))+";"); 
 eval("a"+num2str(ik)+"="+string(localvalues(ik,2))+";") ;
 eval("d"+num2str(ik)+"="+string(localvalues(ik,3))+";") ;
 eval("theta"+num2str(ik)+"="+string(localvalues(ik,4))+";") ;
 alphai=localvalues(ik,1);
 ai=localvalues(ik,2);
 di=localvalues(ik,3);
 thetai=localvalues(ik,4);
 subs(Aiqi)
end
dh=subs(dh)
% 
dht = getDHforTB(dh,'alpha')
comau = SerialLink(dht)
for i_=1:NJoints
   comau.links(i_).A(q(i_))
end
comau.teach([0 0 0 0])