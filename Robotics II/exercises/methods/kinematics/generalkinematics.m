function [V,W,A,Aw] = generalkinematics(J,P,method,ql)
% solveV=0;
assume(ql,'real');
% n=size(fieldnames(ql),1)
n=size(ql,1);
q=sym('q',[n,1],'real');

J=subs(J,ql,q);
P=subs(P,ql,q);

var_ind = 't';
var_dep = 'q';
var_set =[];
qd=sym('qd',[n,1],'real');
qdd=sym('qdd',[n,1],'real');

for i=1:n
    var_dep_str = strcat(var_dep,num2str(i),'(',var_ind,')');
    syms(var_dep_str);
    var_set = [var_set; str2sym(var_dep_str)]; % appends the variable to the set
end
qt = var_set;
assume(qt,'real');
Jt=subs(J,q,qt);
Pt=subs(P,q,qt);
if(method==1)
    
    Jp=J(1:3,:);
    Jo=J(4:end,:);
    Jpt=Jt(1:3,:);
    Jot=Jt(4:end,:);
    V=Jp*qd;
    W=Jo*qd;
    Jpdt=diff(Jpt,t);
    Jpd=simplify(subs(Jpdt,diff(qt,t),qd));
    Jpd=simplify(subs(Jpd,qt,q));
    
    Jodt=diff(Jot,t);
    Jod=simplify(subs(Jodt,diff(qt,t),qd));
    Jod=simplify(subs(Jod,qt,q));
    
    A=Jp*qdd+Jpd*qd;
    Aw=Jo*qdd+Jod*qd;%no estoy seguro de esto
    %terminar para el resto xd
else
    Vt=diff(P,t);
    V=simplify(subs(Vt,diff(qt,t),qd));
    V=simplify(subs(V,qt,q));
    
    W=[]
    
    At=diff(Vt,t);
    A=simplify(subs(At,diff(diff(qt,t)),qdd));
    A=simplify(subs(A,diff(qt,t),qd));
    A=simplify(subs(A,qt,q));
  
end
% 
% if(wsolv==1)
%     if(~isempty(V_))
%         
%         gsol=solve(V==V_)
%     end
% end
end

