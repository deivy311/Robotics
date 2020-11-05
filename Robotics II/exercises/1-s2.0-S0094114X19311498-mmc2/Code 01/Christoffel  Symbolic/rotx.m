function T=rotx(t)
T=zeros(4,4);
T(4,4)=1;

c=cos(t);
s=sin(t);
R=[1 0 0;
    0 c -s;
    0 s c];
T(1:3,1:3)=R;
end