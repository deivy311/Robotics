function [M ] = getM(dhTable, sigma, q,qd, l, m, I,d, isMotor, qdm)
%   Detailed explanation goes here
[KE, T] = getKEwithJacobian(dhTable, sigma, q, qd, l, m, I, d, isMotor)
outputArg2 = inputArg2;
end

