function T=rotx(t)
T=zeros(4,4);
T(4,4)=1;

c=cos(t);
s=sin(t);
R=[c 0 s;
    0 1 0;
    -s 0 c];
T(1:3,1:3)=R;
end