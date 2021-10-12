function [J_pseudo] = Pseudo_Inverse(J,num_digits)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if(~exist('num_digits'))
    num_digits=6;
end
if(~all(J==0))
    
    J_pseudo=round(pinv(J),6);
else
    display("Pay Attention, all zeros in Jacobian, while computing the Pseudo Inverse");
    J_pseudo=J';
end
end

