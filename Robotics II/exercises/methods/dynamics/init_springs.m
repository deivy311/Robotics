function z=init_springs(z)
    z.springs=[];
    if(isfield(z,'k'))
        k=z.k;
    else
        k=sym('k_',[z.n+1,1],'real');
        z.k=k;
    end
    
    if(isfield(z,'b'))
                b=z.b;
    else
        b=sym('b_',[z.n+1,1],'real');
        z.b=b;
    end

    if(isfield(z,'m_spring'))
            m=z.m_spring;
    else
        m=sym('m_',[z.n,1],'real');
        z.m_spring=m;
    end

    if(isfield(z,'q'))
                x=z.q;
    else
        x=z.q;%sym('x_',[z.n,1],'real');
        z.x=q;
    end
    if(isfield(z,'F'))
            F=z.F;
    else
        F=sym('F_',[z.n,1],'real');
        z.F=F;
    end

    for i=1:z.n
        z.springs(i).x_r_dot    =0;
        z.springs(i).x_r_ddot   =0;
        z.springs(i).x_l_dot    =0;
        z.springs(i).x_l_ddot   =0;
        if i==1%i<z.n && i<z.n
            z.springs(i).x_l=0;
            if z.n==1
                z.springs(i).x_r=0;
            else
                z.springs(i).x_r        =z.q(i+1);
                z.springs(i).x_r_dot    =z.q_dot(i+1);
                z.springs(i).x_r_ddot   =z.q_ddot(i+1);
            end
        elseif i==z.n
            z.springs(i).x_r=0;
            z.springs(i).x_l=z.q(i-1);
            z.springs(i).x_l_dot=z.q_dot(i-1);
            z.springs(i).x_l_ddot=z.q_ddot(i-1);

%             if z.n==1
%                 z.springs(i).x_r=0;
%             else
%                 z.springs(i).x_r=z.q(i);
%             end
        else

            z.springs(i).x_l=z.q(i-1);
            z.springs(i).x_l_dot    =z.q_dot(i-1);
            z.springs(i).x_l_ddot   =z.q_ddot(i-1);

            z.springs(i).x_r=z.q(i+1);
            z.springs(i).x_r_dot    =z.q_dot(i+1);
            z.springs(i).x_r_ddot   =z.q_ddot(i+1);
        end
        
        z.springs(i).k_r=k(i+1);
        z.springs(i).k_l=k(i);
        z.springs(i).b_r=b(i+1);
        z.springs(i).b_l=b(i);
        z.springs(i).x=z.q(i);
        z.springs(i).x_dot  =z.q_dot(i);
        z.springs(i).x_ddot =z.q_ddot(i);
        z.springs(i).m=m(i);
        z.springs(i).F=F(i);

        %computing spring forces
        z.springs(i).F_=compute_spring_model(z.springs(i));
    end
end