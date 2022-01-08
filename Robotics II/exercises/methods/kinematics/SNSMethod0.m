function [qdotstarFinal] = SNSMethod0(xdot,J_,q_dMax_,abs_on,autodetectsign)
% SNS
% xdot= V_%[4,2].'
if(~exist('abs_on'))
    abs_on=false;
    disp("Assuming qN positive if it is not in this way comment this ""if""   PAY ATTENTION!")
end
if(~exist('autodetectsign'))
    autodetectsign=false;
    disp("Assuming autodetection as false if it is not in this way comment this ""if""   PAY ATTENTION!")
end
[nx,ny]=size(xdot);
qnum=length(q_dMax_);
qdotPS = eval(pinv(J_))*xdot
maxvalue = 1;
qdotstar = qdotPS
qdot_sns = qdotPS;

if(autodetectsign)
negativeslocales=qdotPS<0
disp("converting q_dot as negative PAY ATTENTION!!!!! abs=false also")
abs_on=false
q_dMax_=-abs(q_dMax_)
end
Jtemp = J_;
Jiter_V=zeros(nx,ny);
originalpos=1:qnum;
qdotstarFinal=q_dMax_;
q_dMaxTemp_=q_dMax_;
numiter=1;
while maxvalue > 0
    [maxvalue, maxindex] = max(abs(qdotstar)-abs(q_dMaxTemp_)); % The positive values are violating the limit. We take the highest one.
       if any(maxvalue > 0) %triying with thsi aproach
%     if (maxvalue > 0) % if there is an exceeding value
        disp("Max value that violates is:  "+ originalpos(maxindex)+"  with value:  "+string(vpa(maxvalue)))
       
        disp("SNS Iterarion:  "+numiter)
%         s = q_dMaxTemp_(1)/qdotstar(maxindex);
        %xdot_sns  = xdot - J(:,maxindex)* sign(qdotPS(maxindex))*V(maxindex)
        %Same as
        if(abs_on)
        Jiter_V=abs(Jtemp(:,maxindex)*q_dMaxTemp_(maxindex));% va con abs o no
        else
        Jiter_V=Jiter_V+(Jtemp(:,maxindex)*q_dMaxTemp_(maxindex));% va con abs o no
        end
        
        xdot_sns  = vpa(xdot - Jiter_V);
        vpa(xdot_sns)
        %         Jtemp = J_;
        q_dMaxTemp_(maxindex)=[];
        disp("Jtemp #:  "+originalpos(maxindex));
        Jtemp(:,maxindex) = []
        disp("Jtemp pseudo inverse #:  "+originalpos(maxindex));
        psJ_=vpa(pinv(Jtemp))
        originalpos(maxindex)=[];
        qdot_components = eval(psJ_*xdot_sns)'
%         qdot_components = eval(simplify(pinv(Jtemp))*xdot_sns)
%         qdot_sns = [qdot_components(1:maxindex-1), q_dMax_(maxindex), qdot_components(maxindex:end)]'
        qdot_sns = [qdot_components(1:maxindex-1), [], qdot_components(maxindex:end)]'
        qdotstar = qdot_sns;
    else
        disp("[INFO] No values are exceeding the limits.")
        disp('Result:')
        qdotstarFinal(originalpos)=qdotstar;
        qdotstarFinal
    end
end
end


