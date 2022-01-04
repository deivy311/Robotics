function [qdotstarFinal] = SNSMethod1(xdot,J_,q_dMax_,abs_on,autodetectsign)
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
J_q_dot_N=zeros(nx,ny);
originalpos=1:qnum;
qdotstarFinal=q_dMax_;
q_dMaxTemp_=q_dMax_;
numiter=1;
while maxvalue > 0
    [maxvalue, maxindex] = max(abs(qdotstar)-abs(q_dMaxTemp_)); % Current positive values violating the limits. We take the highest one.
       if any(maxvalue > 0) %triying with thsi aproach
%     if (maxvalue > 0) % if there is an exceeding value
        disp("Max value that violates is:  "+ originalpos(maxindex)+"  with value:  "+maxvalue)
       
        disp("SNS Iterarion:  "+numiter)
%         s = q_dMaxTemp_(1)/qdotstar(maxindex);
        %xdot_diff  = xdot - J(:,maxindex)* sign(qdotPS(maxindex))*V(maxindex)
        %Same as
        if(abs_on)
        J_q_dot_N=abs(Jtemp(:,maxindex)*q_dMaxTemp_(maxindex));% va con abs o no
        else
        J_q_dot_N=J_q_dot_N+(Jtemp(:,maxindex)*q_dMaxTemp_(maxindex));% va con abs o no
        end
        
        xdot_diff  = xdot - J_q_dot_N;
        vpa(xdot_diff)
        %         Jtemp = J_;
        
        disp("Jtemp #:  "+originalpos(maxindex));
        Jtemp(:,maxindex) = []
        disp("Jtemp pseudo inverse #:  "+originalpos(maxindex));
        psJ_=vpa(pinv(Jtemp))
        originalpos(maxindex)=[];
        qdot_components = eval(psJ_*xdot_diff)'
%         qdot_components = eval(simplify(pinv(Jtemp))*xdot_diff)
%         qdot_sns = [qdot_components(1:maxindex-1), q_dMax_(maxindex), qdot_components(maxindex:end)]'
        qdot_sns = [qdot_components(1:maxindex-1), [], qdot_components(maxindex:end)]'
        qdotstar = qdot_sns;

        %redoing q limitsvalues
        q_dMaxTemp_(maxindex)=[];
    else
        disp("[INFO] No values are exceeding the limits.")
        disp('Result:')
        qdotstarFinal(originalpos)=qdotstar;
        qdotstarFinal
    end
end
end

