function T=GetKenimaticModelAccelerated(a,alfa,q,d)
n=max(size(a));
T=zeros(3,4,n);
T(:,:,1)=JointTransform(a(1),alfa(1),q(1),d(1));
temp=zeros(3,4);
for i=2:n
temp=JointTransform(a(i),alfa(i),q(i),d(i));
T(1:3,1:3,i)=T(1:3,1:3,i-1)*temp(1:3,1:3);
T(1:3,4,i)=T(1:3,4,i-1)+T(1:3,1:3,i-1)*temp(1:3,4);
end
end

function t=JointTransform(a,alfa,q,d)
t=zeros(3,4);
t(1,1)=cos(q);t(1,2)=-sin(q);t(1,4)=a;
t(2,1)=sin(q)*cos(alfa);t(2,2)=cos(q)*cos(alfa);t(2,3)=-sin(alfa);t(2,4)=-sin(alfa)*d;
t(3,1)=sin(q)*sin(alfa);t(3,2)=cos(q)*sin(alfa);t(3,3)=cos(alfa);t(3,4)=cos(alfa)*d;
end