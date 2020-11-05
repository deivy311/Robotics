function [spline_fun,Aw_fun,Aa_fun,Ajerk_fun,MaxsVal] = splinesD(N,NumJoints,t_values,q_values,polg,findmaxsu,Vinit)
NumJoints= 1%solo funciona para 1  no tocar
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Define all variables.
% Number of cubic splines
% N = 2 % No. of points
if exist('findmaxsu')==0
    findmaxsu=true;
end
if exist('Vinit')==0
    Vinit=[0,0];
end
M = N-1 % No. of splines = # points - 1

q = sym('q', [N,NumJoints]);
t = sym('t', [N,1]);
% We'll denote [qA, qB, qC] (material notation) = [Q1, Q2, Q3] (Matlab friendly notation)
% In uppercase to not confuse with the actual points. Althought many of
% them are equivalent

Q = sym('Q', [M,NumJoints]); % Angle value
Qd = sym('Qd', [M,NumJoints]); % Angular velocity
Qdd = sym('Qdd', [M,NumJoints]); % Angular acceleration
Qddd = sym('Qddd', [M,NumJoints]); % Angular acceleration

v = sym('v', [M,NumJoints]); % Speed in splines unions
a = sym('a', [M,NumJoints]); % aceleration in splines unions

%ak = sym('ak', [M,1]); 
syms t_ % Time variable. Underscore to avoid confusion with symvector t


% polg=5;
ak_ = sym('ak_',[polg NumJoints]); % 3 because it's the number of coefficients of a cubic spline.


for i = 1:M
    tau = (t_-t(i))/(t(i+1)-t(i));
    Q(i,:) = q(i,:);
    for p_i=1:polg
    Q(i,:) =Q(i,:) +ak_(p_i,:)*(tau^(p_i)) ;
    end
%     Q(i) = q(i) + ak_(1)*tau + ak_(2)*tau^2 + ak_(3)*tau^3 % Spline equation
    Qd(i,:) = diff(Q(i,:),t_); % Get angular speed.
    Qdd(i,:) = diff(Qd(i,:),t_); % left to display the intermediate step.


    
    % Rest-to-rest conditions
    if i == 1 %% be careful with this
        v_init = Vinit(1)%zeros(1,NumJoints)
        a_init = zeros(1,NumJoints) %% for the cubic case
    else
        v_init = v(i)
        a_init = a(i) %% for the cubic case

    end
    
    if i == M
        v_end =Vinit(2)%zeros(1,NumJoints)
        a_end =zeros(1,NumJoints)
    else
        v_end = v(i+1,:)
        a_end = a(i+1,:)
    end
    
    % For sytstem of equations
    eq_k(i,1:NumJoints,1) = subs(Q(i,:),t_,t(i+1)) == q(i+1,:); % At the end of the interval, the position is the next q.
    eq_k(i,:,2) = simplify(subs(Qd(i,:),t_,t(i))) == v_init; % Beggining of interval, tau = 0, match last velocity
    eq_k(i,:,3) = simplify(subs(Qd(i,:),t_,t(i+1))) == v_end; % End of interval, tau = 1, match next velocity
    if polg==5
%     else
%     eq_k(i,1) = subs(Q(i),t_,t(i+1)) == q(i+1); % At the end of the interval, the position is the next q.
%     eq_k(i,2) = simplify(subs(Qd(i),t_,t(i))) == v_init; % Beggining of interval, tau = 0, match last velocity
%     eq_k(i,3) = simplify(subs(Qd(i),t_,t(i+1))) == v_end ;% End of interval, tau = 1, match next velocity
    eq_k(i,:,4) = simplify(subs(Qdd(i,:),t_,t(i))) == a_init ;% Beggining of interval, tau = 0, match last velocity
    eq_k(i,:,5) = simplify(subs(Qdd(i,:),t_,t(i+1))) == a_end ;% End of interval, tau = 1, match next velocity
    end
   
    
    ak(i) = solve(eq_k(i,:).',ak_)
    
    %solving
    Qd(i,:) = simplify(subs(Qd(i,:),ak(i)));
    Qdd(i,:) = simplify(subs(Qdd(i),ak(i)));
    Qddd(i,:) = diff(Qdd(i,:),t_); % left to display the intermediate step.
    Qddd(i,:) = simplify(subs(Qddd(i,:),ak(i)));
%     if i > 1
%         Qdd_end1 = subs(Qdd(i-1),t_,t(i)) % End of first segment
%         Qdd_init2 = subs(Qdd(i),t_,t(i)) % Beggining of sencond segment
%         v_eq(i) = simplify(Qdd_end1 == Qdd_init2)
%         
%         %v_sol(i) = simplify(solve(Qdd_end1 == Qdd_init2, v(i))) 
%         if polg==5
%                 Qddd_end1 = subs(Qddd(i-1),t_,t(i)) % End of first segment
%                 Qddd_init2 = subs(Qddd(i),t_,t(i)) % Beggining of sencond segment
%                 a_eq(i) = simplify(Qddd_end1 == Qddd_init2)
%         end
%     end

end
printStruct(ak)

%     t_values = [ 0,3]
%[0,3,6];
%     q_values = [pi/4,-pi/2]*180/pi;
% t_values = [ 1, 2.2324460431654676258992805755396, 2.8486690647482014388489208633094, 4.6973381294964028776978417266187]%[1,2,2.5,4];%[1,2.2324,2.8487,4.6973];%
% q_values = [45,90,-45,45];
    vars = [t,q];
    
    values = [t_values; q_values].';
if M<2 %to avoid empy states%
    %             Qdd_end1 = subs(Qdd(i-1),t_,t(i)) % End of first segment
    %             Qdd_init2 = subs(Qdd(i),t_,t(i)) % Beggining of sencond segment
    v_eq(i,:) = Qd(i,:);
    a_eq(i,:) = Qdd(i,:);
    
else
    for i = 1:M
        %     Qdd(i) = diff(Qd(i),t_) % left to display the intermediate step.
        %     Qdd(i) = simplify(subs(Qdd(i),ak(i)))
        
        
        if i > 1
            if polg==5
            Qddd_end1 = subs(Qddd(i-1,:),t_,t(i)); % End of first segment
            Qddd_init2 = subs(Qddd(i,:),t_,t(i)); % Beggining of sencond segment
            a_eq(i,:) = simplify(Qddd_end1 == Qddd_init2);
%             else
            end
            Qdd_end1 = subs(Qdd(i-1,:),t_,t(i)); % End of first segment
            Qdd_init2 = subs(Qdd(i,:),t_,t(i)); % Beggining of sencond segment
            v_eq(i,:) = simplify(Qdd_end1 == Qdd_init2);
            %v_sol(i) = simplify(solve(Qdd_end1 == Qdd_init2, v(i)))
%             end
        end
        
    end
    % 
    if polg==5
    av_sol_sym = solve([a_eq,v_eq], [a,v] ); % solve for speed at the joint of segments
    
%     v_eq1= subs(v_eq, a)
    printStruct(av_sol_sym)
    avnames = fieldnames(av_sol_sym);
    av_sol_sym2=struct2cell(av_sol_sym);
    
    a_sol_sym=cell2struct(av_sol_sym2(1:M),avnames(1:M));
    v_sol_sym=cell2struct(av_sol_sym2(M+1:end),avnames(M+1:end));
%     vnames = fieldnames(v_sol_sym)
    anames = fieldnames(a_sol_sym)
    a_sol = zeros(1,M);
%     v_sol = zeros(1,M);
    for i = 1:length(anames)
        a_sol(i) = eval(subs(getfield(a_sol_sym,anames{i}),vars,values))
    end
    else
    v_sol_sym = solve(v_eq, v) % solve for speed at the joint of segments
    end
    printStruct(v_sol_sym)

%     t_values = [0,3];%[1,2.2324,2.8487,4.6973];%
%     q_values = [pi/4,-pi];
%     vars = [t,q]
%     
%     values = [t_values; q_values].'
    names = fieldnames(v_sol_sym)
    v_sol = zeros(1,M);
    for i = 1:length(names)
        v_sol(i) = eval(subs(getfield(v_sol_sym,names{i}),vars,values))
    end
%     end
end


%get splines functions
% Get splines functions
spline_peak=[];
Aw_peak=[];
Aa_peak=[];
Ajerk_peak=[];
spline_timepeak=[];
Aw_timepeak=[];
Aa_timepeak=[];
Ajerk_timepeak=[];
for i = 1:M
    Q_ = subs(Q(i,:),ak(i)); % substitute found values
    if M>1
        if polg==5
            Q_ = vpa(simplify(subs(Q_,[a,v],[a_sol.',v_sol.'])));
            
        else
            
            Q_ =vpa(simplify( subs(Q_,v,v_sol.')));
        end
        
    end
    spline_fun(i,:) = subs(Q_, vars, values)
    %%modified by DAvid
    Aw_fun(i,:)=diff(spline_fun(i,:),t_);
    Aa_fun(i,:)=diff(Aw_fun(i,:),t_);
    Ajerk_fun(i,:)=diff(Aa_fun(i,:),t_);
    
    if(findmaxsu)
    %searching maximus and minumus
    spline_M_=vpasolve(spline_fun(i,:)==0,[t_values(i),t_values(i+1)]);
    Aw_M_=vpasolve(Aw_fun(i,:)==0,[t_values(i),t_values(i+1)]);
    Aa_M_=vpasolve(Aa_fun(i,:)==0,[t_values(i),t_values(i+1)]);
    Ajerk_M={};
    if( polg==5)
        Ajerk_M_=vpasolve(Ajerk_fun(i,:)==0,[t_values(i),t_values(i+1)]);
        if(isempty(Ajerk_M_)==0)
            %         Aa_M=[Aa_M;Aa_M_];
            Ajerk_M{i}=Ajerk_M_;
            Aa_peak=[Aa_peak,subs(Aa_fun(i),t_,Ajerk_M{i}')];
            Aa_timepeak=[Aa_timepeak,Ajerk_M{i}'];
        end
    end
    
    %% assigning
    if(isempty(spline_M_)==0)
%         spline_M=[spline_M;spline_M_];
        spline_M{i}=spline_M_;

    end
    if(isempty(Aw_M_)==0)
%         Aw_M=[Aw_M;Aw_M_];
        Aw_M{i}=Aw_M_;
        spline_peak=[spline_peak,subs(spline_fun(i),t_,Aw_M{i}')];
        spline_timepeak=[spline_timepeak,Aw_M{i}'];

    end
    if(isempty(Aa_M_)==0)
%         Aa_M=[Aa_M;Aa_M_];
        Aa_M{i}=Aa_M_;
        Aw_peak=[Aw_peak,subs(Aw_fun(i),t_,Aa_M{i}')];
        Aw_timepeak=[Aw_timepeak,Aa_M{i}'];

    end
%         if(isempty(Ajerk_M_)==0)
%         Ajerk_M=[Ajerk_M;Ajerk_M_];
%     end

    end
end
MaxsVal.timepeak.Aw=Aw_timepeak;
MaxsVal.timepeak.spline=spline_timepeak;
MaxsVal.timepeak.Aa=Aa_timepeak;
MaxsVal.PeakValue.spline=spline_peak;
MaxsVal.PeakValue.Aw=Aw_peak;
MaxsVal.PeakValue.Aa=Aa_peak;
end

