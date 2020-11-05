%Inverse problem of representing a rotation matrix with a sequence of three ZXZ Euler angles.

%Author: Andrés Arciniegas
clear; clc;

Ra = [ 0.4330   -0.5        -0.75;
       0.25     0.8660      -0.4330;
       0.8660   0            0.5];

Rb = [ 0.5950      0.5403     0.5950
       -0.3821    -0.8415    -0.3821
       -1/sqrt(2)   0         1/sqrt(2) ]  ; %This is not a rotational matrix, as its norm is not equal to 1
   
Rc = [-1/sqrt(2)  -1/sqrt(2)   0
      1/sqrt(2)   -1/sqrt(2)   0
      0           0            1];

RR = {Ra Rb Rc};
                
R = sym('r%d%d',[3 3]) %

%Angles
theta = [atan2(sqrt(R(1,3)^2 + R(2,3)^2),R(3,3)); ...
         atan2(-sqrt(R(1,3)^2 + R(2,3)^2),R(3,3))];
     
psi = [atan2(R(3,1)/sin(theta(1)), R(3,2)/sin(theta(1))); ...
       atan2(R(3,1)/sin(theta(2)), R(3,2)/sin(theta(2)))];
   
phi = [atan2(R(1,3)/sin(theta(1)), -R(2,3)/sin(theta(1))); ...
       atan2(R(1,3)/sin(theta(2)), -R(2,3)/sin(theta(2)))];

angles = [phi theta psi];

%anglesNum = eval(subs(angles, R, Rb))

for i = 1:3
    fprintf("\nRotacional Matrix #" + i + '\n')
    try 
        if (norm(RR{i})~= 1)
            fprintf(2,"-- Error: Matrix is not normal. Not a rotational matrix.\n")
        elseif(norm(Rc.'-inv(Rc))>1e-12) %Tolerance. Floating point problems comparing with isequal(,)
            fprintf(2,"-- Error: Matrix is not ortogonal. Not a rotational matrix.\n")
        else
            if(isequal(eval(subs(theta, R, RR{i})), [0;0]))
                fprintf(2,"-- Value of Theta equals to zero. It is a singularity.\n")
                %There are infinite solutions.
            else
                fprintf("Two solutions shown.\n")
                fprintf('    phi       theta     psi\n')
                anglesNum = eval(subs(angles, R, RR{i}));
                disp(anglesNum)
            end
        end
    catch ex
          disp(ex)
    end
end