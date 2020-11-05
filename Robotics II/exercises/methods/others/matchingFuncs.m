function [D2_,Dparam_,DNames_] = matchingFuncs(D2,Dparam1,Dparam2,DNames1,DNames2)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% for i=2:size(Dfunc)
[C11,C12]=ismember(Dparam1,Dparam2,'rows');
[C21,C22]=ismember(Dparam2,Dparam1,'rows');
% C1=C1((Dparam1(C1)-Dparam2)==0)%esto no funciona
maxlength=length(Dparam1)+length(Dparam2)-length(Dparam1(C11));
maxsizegy=size(Dparam1)+size(Dparam2)-size(Dparam1(C11));

minsize=length(Dparam1)+1;

a=sym('a',[maxsizegy],'real');
a2temp=sym('a2temp',[size(DNames2)],'real');

disp("visual match D1match")
Dparam1(C11)
disp("D2")
Dparam2
D2_=subs(D2,DNames2,a2temp);

D2_=subs(D2_,a2temp(~C21),DNames1(minsize:maxlength));

D2_=subs(D2_,a2temp(C21),DNames1(C22));
Dparam_=[Dparam1;Dparam2(~C21)]
DNames_=a;
% end
end

