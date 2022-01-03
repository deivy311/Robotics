function spring_model=compute_spring_model(t_spring)
    
    
    m=t_spring.m;
    
    x=t_spring.x;
    x_dot   =t_spring.x_dot;
    x_ddot  =t_spring.x_ddot;
    
    x_r=t_spring.x_r;
    x_r_dot   =t_spring.x_r_dot   ;
    x_r_ddot  =t_spring.x_r_ddot  ;

    x_l=t_spring.x_l;
    x_l_dot   =t_spring.x_l_dot   ;
    x_l_ddot  =t_spring.x_l_ddot  ;

    k_r=t_spring.k_r;
    k_l=t_spring.k_l;
    b_r=t_spring.b_r;
    b_l=t_spring.b_l;
    spring_model=m*x_ddot+b_l*(x_dot-x_l_dot)+b_r*(x_dot-x_r_dot)+k_l*(x-x_l)+k_r*(x-x_r);
end