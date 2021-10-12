function [TTotal TPartial,RTotal,RPartial,PTotal, PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]= getTransformationMatrixConfigbyangle(sigmaD,l,q,dc,onshortnotation,angle_,prismatic_CoM_method,xyx_offset_local,z)
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

firstParam= 'alpha';
alpha = sym('alpha');
a =  sym('a');
d = sym('d');
theta =sym('theta');
sigmaD=  upper(sigmaD);
sigmaD(find(sigmaD=='P'))=1 ;
sigmaD(find(sigmaD=='R'))=0 ;
joints = size(sigmaD,2); %Number of joints
if(~exist("angle_"))
    angle_=sym(zeros(1,joints))
    %     switch sigmaD(2,i)
    %     case 1*'X'
    %         angle_=0
    %     case 1*'Y'
    %         angle_=pi/2
    %     case 1*'Z'
    %         %angle_=0
    %          ME = MException('case in Z axis not programmed!!1');
    %          throw(ME)
    %     otherwise
    %angle_=sym('alpha','real')
    %     end
end
if ~exist('firstParam','var')
    % third parameter does not exist, so default it to something
    firstParam = 'a';
end
if ~exist('onshortnotation','var')
    % third parameter does not exist, so default it to something
    onshortnotation = false;
end
if ~exist('prismatic_CoM_method','var')
    % third parameter does not exist, so default it to something
    prismatic_CoM_method = 1;
end

if strcmp(firstParam,'a')
    params = [a alpha d theta];
end
if strcmp(firstParam,'alpha')
    params = [alpha a d theta];
end


A  = [cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta)  a*cos(theta);...
    sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta)  a*sin(theta);...
    0     sin(alpha)            cos(alpha)              d          ;...
    0               0                   0               1         ];


%Loop row by row, calculating each matrix, storing in DH
offsetJoints=zeros(joints,1);%the key
ThetaPartial=sym(offsetJoints);
actualaxis=['X'*1];
if(sigmaD(1,1)==1)
    actualaxis=['Z'*1] ;
end
if(z.global_q_reference)
    [params1,Ttemp,ThetaPartial,offsetJoints,anglerot] = localrotationsbyangle(z.q_global,l,sigmaD,actualaxis,angle_);
    
else
    [params1,Ttemp,ThetaPartial,offsetJoints,anglerot] = localrotationsbyangle(q,l,sigmaD,actualaxis,angle_);
end
for i = 1:joints
    %     tempR{1}=eye(3);
    %     tempR{2}=eye(3)
    %     Ttemp=eye(4);
    
    %     if(sigmaD(1,i)==1)
    %         params1 = [pi/2    0       q(i)    pi] ;
    %         switch sigmaD(2,i)
    %             case 'X'
    %                 %         offsetJoints=[0,pi/2,0]';%the key
    %                 if(i<joints)
    %                     if(sigmaD(1,i+1)==0)
    %                        offsetJoints(i+1)=pi/2; %probar esto no s'e si funciona
    %                     end
    %                 end
    %                 %                 [tempR{1}] = getEulerRotMatProduct('x',pi/2);
    %                 %                 [tempR{1}] = tempR{1}*getEulerRotMatProduct('y',pi/2);
    %                 [tempR{1}] = getEulerRotMatProduct('xy',[pi/2,pi/2]);
    %                 Ttemp(1:end-1,1:end-1)=tempR{1};
    %                 %         case 'Y'
    %                 %              [tempR] = getEulerRotMatProduct('y',pi/2);
    %                 %         case 'Z'
    %                 %             [tempR] = getEulerRotMatProduct('y',pi/2);
    %         end
    %     else
    %         params1 = [ 0    l(i)   0    q(i)+offsetJoints(i)] ;
    %         ThetaPartial(i)=q(i);
    %     end
    %
    
    TPartial{i} = simplify(subs(A, params, params1{i}));
    TPartial{i}=anglerot{i}*Ttemp{i}*TPartial{i};
    
    % the order matters!!
    if(~isempty(xyx_offset_local))
        display("PAY!!!!!!!!!!! attention!!!!! offset in axis is different of zero")
        local_trans_offset=sym(eye(4));
        local_trans_offset(1:end-1,end)=xyx_offset_local(:,i);
        TPartial{i}=local_trans_offset*TPartial{i};
    end
    
    
    RPartial{i}=TPartial{i}(1:end-1,1:end-1);
    PPartial{i}=TPartial{i}(1:end-1,end);
    
    
    %     TPartial{i}(1:end-1,1:end-1)= RPartial{i};
    %     TPartial{i}(1:end-1,end)=PPartial{i};
end
%Chain multiplication
% I = sym(zeros(3,3,n));
% rc = [];
% for i = 1:n
%     rc = [rc [-l(i)+z.dc(i); 0; 0]];
%     I(:,:,i) = diag(sym(strcat({'Ixx','Iyy','Izz'},int2str(i))));
% end
TTotal = TPartial{1};
RTotali{1}=RPartial{1};
PTotali(:,1)=PPartial{1};
% Pc(:,1)=subs(PPartial{1},l(1),z.dc(1));
i=1;
if (z.dc_method==1)
    
    
    %method based on cartesian
    %z.dc=[rcx1,rcy1,rcz1;...;rcxn,rcyn,rczn;]'
    rc_temp=zeros(z.n,1);
    if any(has(TTotal(1:end-1,end),l(i)))
        rc_temp=RTotali{i}*[[l(i),0,0]'+z.dc(:,i)];
        Pc(:,i)=simplify(rc_temp);
    else
        Pc(:,1)=simplify(TTotal(1:end-1,end));
    end
    
else
    if sigmaD(1,1)==1 %% for the moment it is only available for revolutes
        if prismatic_CoM_method(i)==1
            Pc(:,1)=simplify(subs(TTotal(1:end-1,end),l(i),z.dc(i)));
        else
            Pc(:,1)=simplify(subs(TTotal(1:end-1,end),q(i),q(i)-z.dc(i)));
        end
        
    else
        Pc(:,1)=simplify(subs(TTotal(1:end-1,end),l(i),z.dc(i)));
    end
end
for i = 2:joints
    TTotal = TTotal*TPartial{i};
    RTotali{i}=simplify(RTotali{i-1}*RPartial{i});
    if (z.dc_method==1)
        %method based on cartesian
        %z.dc=[rcx1,rcy1,rcz1;...;rcxn,rcyn,rczn;]'
        rc_temp=zeros(z.n,1);
        if any(has(TTotal(1:end-1,end),l(i)))
            rc_temp=RTotali{i}*[[l(i),0,0]'+z.dc(:,i)];
        
            Pc(:,i)=simplify(PTotali(:,i-1)+rc_temp);
        else
            Pc(:,i)=simplify(TTotal(1:end-1,end));
        end
    else
        if sigmaD(1,i)==1 %% for the moment it is only available for revolutes
            if prismatic_CoM_method(i)==1
                Pc(:,i)=simplify(subs(TTotal(1:end-1,end),l(i),z.dc(i)));
            else
                Pc(:,i)=simplify(subs(TTotal(1:end-1,end),q(i),q(i)-z.dc(i)));
            end
        else
            Pc(:,i)=simplify(subs(TTotal(1:end-1,end),l(i),z.dc(i)));
        end
    end
    PTotali(:,i)=simplify(TTotal(1:end-1,end));
    
    
end
ThetaTotal=sum(ThetaPartial);
%Simplify result
if(onshortnotation)
    sincos_ = getShortNotation_SinCos(joints);
    % % Jl_ = toShortNotation(Jl,sincos_)
    TTotal = toShortNotation(simplify(TTotal),sincos_);
    RTotal= TTotal(1:end-1,1:end-1);%toShortNotation(TTotal(1:end-1,1:end-1),sincos_);
    PTotal= TTotal(1:end-1,end);%toShortNotation(TTotal(1:end-1,end),sincos_);
    Pc=toShortNotation(Pc,sincos_);
    PTotali=toShortNotation(PTotali,sincos_);
else
    TTotal = simplify(TTotal);
    RTotal= TTotal(1:end-1,1:end-1);
    PTotal= TTotal(1:end-1,end);
end
end
