function [q_d1,q_d2,Jp,Jpt] = general_inv_kinematics(J,V,method,ql,A)
% solveV=0;
assume(ql,'real');
% n=size(fieldnames(ql),1)
n=size(ql,1);
q=sym('q',[n,1],'real');

J=subs(J,ql,q);
V=subs(V,ql,q);%test this

var_ind = 't';
var_dep = 'q';
var_set =[];
qd=sym('qd',[n,1],'real');
qdd=sym('qdd',[n,1],'real');
q_d2=[]
for i=1:n
    var_dep_str = strcat(var_dep,num2str(i),'(',var_ind,')');
    syms(var_dep_str);
    var_set = [var_set; str2sym(var_dep_str)]; % appends the variable to the set
end
qt = var_set;
assume(qt,'real');
Jt=subs(J,q,qt);
% Pt=subs(P,q,qt);
if(method==1)
    
    Jp=J(1:3,:);
%     Jo=J(4:end,:);
    Jpt=Jt(1:3,:);
%     Jot=Jt(4:end,:);
    q_d1=Jp\V;
%     W=Jo*qd;
    Jpdt=diff(Jpt,t);
    Jpd=simplify(subs(Jpdt,diff(qt,t),qd));
    Jpd=simplify(subs(Jpd,qt,q));
    if(exist('A')
    q_d2=Jp\(A-Jpd*q);
    end
%     Jodt=diff(Jot,t);
%     Jod=simplify(subs(Jodt,diff(qt,t),qd));
%     Jod=simplify(subs(Jod,qt,q));
    
%     Aw=Jo*qdd+Jod*qd;%no estoy seguro de esto
    %terminar para el resto xd
else
    Jp=pinv(J(1:3,:));
%     Jo=J(4:end,:);
    Jpt=pinv(Jt(1:3,:));
%     Jot=Jt(4:end,:);
    q_d1=Jp\V;
%     W=Jo*qd;
    Jpdt=diff(Jpt,t);
    Jpd=simplify(subs(Jpdt,diff(qt,t),qd));
    Jpd=simplify(subs(Jpd,qt,q));
    if(exist('A')
    q_d2=Jp\(A-Jpd*q);
    end
%     Jodt=diff(Jot,t);
%     Jod=simplify(subs(Jodt,diff(qt,t),qd));
%     Jod=simplify(subs(Jod,qt,q));
    
%     Aw=Jo*qdd+Jod*qd;%no estoy seguro de esto
    %terminar para el resto xd
  
end
% 
% if(wsolv==1)
%     if(~isempty(V_))
%         
%         gsol=solve(V==V_)
%     end
% end
end

