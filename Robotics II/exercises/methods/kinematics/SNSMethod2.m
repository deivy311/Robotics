%author David Esteban Imbajoa Ruiz
function [qdotstar,W] = SNSMethod2(xdot,J,qN_Satured,Qdotmin,Qdotdmax,autodetect_sign)
disp("Assuming qN positive if it is not in this way comment this line  PAY ATTENTION!")
% %test it!! if it function doesnt work commetn this if
% if(exist('autodetect_sign','var'))
%     if(autodetect_sign)
%         qN_Satured=sign(pinv(J)*xdot).*qN_Satured;
%     end
% end

% qN_Satured=abs(qN_Satured)
if(~exist('Qdotmin','var'))
Qdotmin=-qN_Satured;
end 
if(~exist('Qdotdmax','var'))
Qdotdmax=qN_Satured;
end 
n=length(qN_Satured)
% %test it!! if it function doesnt work commetn this if
qN_Satured=sym(zeros(n,1));
W=eye(n);
W_star=W;%% no sé aen dónde va esto
qN_Satured_star=qN_Satured;%% no sé aen dónde va esto
s=1;%current task scaled factor
s_star=0;%largest task scale factor so far
limit_exceeded=true;
numiter=0;
while limit_exceeded
    numiter=numiter+1;
    limit_exceeded=false;
    qdotstar=vpa(qN_Satured+pinv(J*W)*(xdot-J*qN_Satured));%antes de entrar en esta función debe ser inicialzada con qdotstar=pinv(J)*xdot
    Qdotdmax_=round(Qdotdmax,4);
    Qdotmin_=round(Qdotmin,4);
    qdotstar_=round(qdotstar,4);
    [maxvalue, maxindex] = max(abs(qdotstar)-abs(Qdotdmax_));%q_dMaxTemp_)); % The positive values are violating the limit. We take the highest one. coloqué Qdotmax pero no sé sie stá bien
%     Violated_index=find(qdotstar>Qdotdmax|qdotstar<Qdotmin);
    Violated_index=find((qdotstar_>Qdotdmax_)|(qdotstar_<Qdotmin_));
    Violatedvalue=false;
    if(~isempty(Violated_index))
        Violatedvalue=true
    end
    if (Violatedvalue ) % if there is an exceeding value
        limit_exceeded=true;
        disp("SNS Iterarion:  "+numiter)
        dnimss=digits();
        digits(4);
        disp("The valued that violate are:  "+ mat2str(string(Violated_index'))+"  with values:  "+mat2str(string(qdotstar(Violated_index)')));
        digits(dnimss);
        a=pinv(J*W)*xdot;
        b=qdotstar-a;
        [ ScalingFactor,MostCriticalJoint]=getTaskScalingFactor(a,b,n,Qdotmin,Qdotdmax);
        if(ScalingFactor>s_star)
            s_star=ScalingFactor;
            W_star=W;
            qN_Satured_star=qN_Satured;
        end
        j=MostCriticalJoint;
        W(j,j)=0;
        if(qdotstar(j)>Qdotdmax(j))
            qN_Satured(j)=Qdotdmax(j);
        end
        if(qdotstar(j)<Qdotmin(j))
            qN_Satured(j)=Qdotmin(j);
        end
%         m=length(xdot);%I'm not sure about this part it is m dimention of the velocity or the rank
          
           
        %         if(rank(J*W)<m)
        
%         m=n-numiter; %number of remaining joints?
%         if(rank(J*W)<=m)%if it doesnt work modify here, removethe '=' 
        m=rank(xdot);
        if(rank(J*W)<m)
            s=s_star;
            W=W_star;
            qN_Satured=qN_Satured_star;
            qdotstar=qN_Satured+pinv(J*W)*(s*xdot-J*qN_Satured);
            limit_exceeded=false;
            disp("the task couldnt be completed check rank of JW or m tal vez lo calculé mal ajaj")
        end
    end
    
    
    if(~limit_exceeded)
        disp('Result:')
        %         qdotstarFinal(originalpos)=qdotstar;
        
    end
    qdotstar
end


end
function [ ScalingFactor,MostCriticalJoint]=getTaskScalingFactor(a,b,n,Qdotmin,Qdotmax)
Smin=vpa((Qdotmin-b)./(a));
Smax=vpa((Qdotmax-b)./(a));
swithindex=find(Smin>Smax);
if(~isempty(swithindex))
    stemp=Smin(swithindex);
    Smin(swithindex)=Smax(swithindex);
    Smax(swithindex)=stemp;
end
s_max=min(Smax);
s_min=max(Smin);
[MostCriticalJointValue,MostCriticalJoint] =min(abs(Smax));
disp("Actual Most Critical Join is:  "+MostCriticalJoint);
ScalingFactor=s_max;
if(s_min>s_max||s_max<0||s_min>1)
ScalingFactor=0;
end
disp("Actual Scaling factor is:  "+string(ScalingFactor));

end




















