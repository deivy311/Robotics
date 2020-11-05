function [TTotal TPartial,RTotal,RPartial,PTotal, PPartial]= getTransformationMatrix(dhTable,firstParam,onshortnotation)
    % Default order: a - alpha - d - theta (Can be changed by the entry firstParam)
    % Must initialize parameter before inserting
    
    % From Denavit-Hartemberg table to Homogeneous transform Matrix

    %qnum = 7; %Number of joints

    %Sym values for depicting undefined variables in result. They can be
    %numbers
    %q = sym('q', [1 qnum]);
    %d = sym('d', [1 qnum]);
    %a = sym('a', [1 qnum]); 
    
    %Define symbolic variables
  
           
    alpha = sym('alpha');
    a =  sym('a'); 
    d = sym('d'); 
    theta =sym('theta');  
    
    if ~exist('firstParam','var')
     % third parameter does not exist, so default it to something
      firstParam = 'a';
    end  
    if ~exist('onshortnotation')
     % third parameter does not exist, so default it to something
      onshortnotation = false;
    end
    if strcmp(firstParam,'a')
        params = [a alpha d theta];
    end
    if strcmp(firstParam,'alpha')
        params = [alpha a d theta];
    end
    
    joints = size(dhTable,1); %Number of joints
    A  = [cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta)  a*cos(theta);...
          sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta)  a*sin(theta);...
                0     sin(alpha)            cos(alpha)              d          ;...
                0               0                   0               1         ];


    %Loop row by row, calculating each matrix, storing in DH
    for i = 1:joints
        params1 = dhTable(i,:); 
        TPartial{i} = subs(A, params, params1);
        RPartial{i}= TPartial{i}(1:end-1,1:end-1);
        PPartial{i}= TPartial{i}(1:end-1,end);

    end
    %Chain multiplication
    TTotal = TPartial{1};
    for i = 2:joints
        TTotal = TTotal*TPartial{i};
    end
    %Simplify result
    if(onshortnotation)
    sincos_ = getShortNotation_SinCos(joints);
% % Jl_ = toShortNotation(Jl,sincos_)
    TTotal = toShortNotation(simplify(TTotal),sincos_);
    RTotal= TTotal(1:end-1,1:end-1);%toShortNotation(TTotal(1:end-1,1:end-1),sincos_);
    PTotal= TTotal(1:end-1,end);%toShortNotation(TTotal(1:end-1,end),sincos_);
    else
    TTotal = simplify(TTotal);
    RTotal= TTotal(1:end-1,1:end-1);
    PTotal= TTotal(1:end-1,end);
    end
end
