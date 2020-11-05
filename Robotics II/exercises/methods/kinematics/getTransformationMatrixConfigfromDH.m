%aurhor David Esteban Imnjoa Ruiz
function [TTotal TPartial,RTotal,RPartial,PTotal, PPartial,Pc,RTotali,ThetaPartial,ThetaTotal,PTotali]= getTransformationMatrixConfigfromDH(dhTable,l,q,dc,onshortnotation,firstParam)
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
ThetaPartial=[];
ThetaTotal=[];
firstParam= 'alpha';
alpha = sym('alpha');
a =  sym('a');
d = sym('d');
theta =sym('theta');
% dhTable=  upper(dhTable);
% dhTable(find(dhTable=='P'))=1 ;
% dhTable(find(dhTable=='R'))=0 ;
if ~exist('firstParam','var')
    % third parameter does not exist, so default it to something
    firstParam = 'alpha';
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
% offsetJoints=zeros(joints,1);%the key
% ThetaPartial=sym(offsetJoints);
% actualaxis=['X'*1];
% if(dhTable(1,1)==1)
% actualaxis=['Z'*1] ;
% end
% [params1,Ttemp,ThetaPartial,offsetJoints] = localrotations(q,l,sigmaD,actualaxis);
for i = 1:joints
    params1 = dhTable(i,:); 
    TPartial{i} = simplify(subs(A, params, params1));
%     TPartialPc{i} = simplify(subs(A, params, params1));
    Ttemp{i}=eye(4);
    TPartial{i}=Ttemp{i}*TPartial{i};
    RPartial{i}=TPartial{i}(1:end-1,1:end-1);
    PPartial{i}=TPartial{i}(1:end-1,end);

end
%Chain multiplication
% I = sym(zeros(3,3,n));
% rc = [];
% for i = 1:n
%     rc = [rc [-l(i)+dc(i); 0; 0]];
%     I(:,:,i) = diag(sym(strcat({'Ixx','Iyy','Izz'},int2str(i))));
% end
TTotal = TPartial{1};
RTotali{1}=RPartial{1};
PTotali(:,1)=PPartial{1};

Pc(:,1)=subs(PPartial{1},l(1),dc(1));
for i = 2:joints
    TTotal = TTotal*TPartial{i};
    RTotali{i}=simplify(RTotali{i-1}*RPartial{i});
    Pc(:,i)=simplify(subs(TTotal(1:end-1,end),l(i),dc(i)));
    PTotali(:,i)=simplify(TTotal(1:end-1,end));

   
end
%  ThetaTotal=sum(ThetaPartial);
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
