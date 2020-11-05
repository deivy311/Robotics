%David esteban Imbajoa Ruiz
function [Je,Ve,Ja,Je_,Ve_,Ja_] = TaskAugmentation(p,nDOF,J,fpath,pathVars,q,qd,varstoreplace,varstoreplace_)
Je=[];
Ve_=[];
J2=jacobian(p,q)
Jpath=gradient(fpath,pathVars)';
Jpath_q=subs(Jpath,pathVars,p);
Ja=simplify(Jpath_q*J2);
Je=[J;Ja];
disp("assuming that la función fp is equal to cero entonces  el último valor de Ve será cero, de otra manera, cambiar Pay Attention!!")
disp("si demora remplazar varaibles o quitar simplify")
Ve=(Je*qd);
if(exist('varstoreplace'))
    Je_=subs(Je,varstoreplace,varstoreplace_)
    Ve_=subs(Ve,varstoreplace,varstoreplace_)
    J_=subs(J,varstoreplace,varstoreplace_)
    Ja_=subs(Ja,varstoreplace,varstoreplace_)
    disp("************************************")
    disp("Rank J_:  "+rank(J_))
    disp("Rank Ja_:  "+rank(Ja_))
    disp("Rank Je_:  "+rank(Je_))
     disp("************************************")
end

disp("Rank J:  "+rank(J))
disp("Rank Ja:  "+rank(Ja))
disp("Rank Je:  "+rank(Je))
end

