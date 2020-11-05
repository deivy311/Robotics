function T=rotz(t)
c=cos(t);
s=sin(t);
T=[c -s 0 0;
    s c 0 0;
    0 0 1 0;
    0 0 0 1];
end