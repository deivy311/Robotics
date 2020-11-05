close all
clear all
clc

NJoints=4

q = sym('q',[1 4])
d = sym('d',[1 4])
dh = [pi/2 0 202 q(1);
        0   210.5 0  q(2);
        0   268  0  q(3);
        0   174.5 0  q(4)]
dht = getDHforTB(dh,'alpha')
comau = SerialLink(dht)
olddigits=digits(4)
for i_=1:NJoints
   vpa(comau.links(i_).A(q(i_)))
end
digits(olddigits)
comau.teach([0 0 0 0])