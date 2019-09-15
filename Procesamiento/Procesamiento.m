%% 102 Z left Ankle
%% 130 Z left Ankle

close all
clc

a = LRFandKinectdataPrueba1S1(:,102);
b = LRFandKinectdataPrueba1S1(:,130);
figure()
plot(a)
hold on
plot(b,'r')

LDD = a - b;

figure()
plot(LDD)

punto1 = LRFandKinectdataPrueba1S1(:,100:102);
punto2 = LRFandKinectdataPrueba1S1(:,128:130);

W=LRFandKinectdataPrueba1S1(:,5);
X=LRFandKinectdataPrueba1S1(:,6);
Y=LRFandKinectdataPrueba1S1(:,7);
Z=LRFandKinectdataPrueba1S1(:,8);

p_base = LRFandKinectdataPrueba1S1(:,2:4);

x2 = X.^2;
y2 = Y.^2;
z2 = Z.^2;
xy = X.*Y;
xz = X.*Z;
yz = Y.*Z;
wx = W.*X;
wy = W.*Y;
wz = W.*Z;

vecX = [2.0 * (xy - wz), 1.0 - 2.0 * (x2 + z2), 2.0 * (yz + wx)];
vecY = [2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (x2 + y2)];
vecZ = [1.0 - 2.0 * (y2 + z2), 2.0 * (xy + wz), 2.0 * (xz - wy)];

[n m] = size(vecX);
D = [];
t=[];
p_sal1=[];
for x=1:n
    D(x) = (-1)*(vecX(x,1)*p_base(x,1) + vecX(x,2)*p_base(x,2) + vecX(x,3)*p_base(x,3));
    t(x) = (-1)*(D(x) + vecX(x,1)*punto1(x,1) + vecX(x,2)*punto1(x,2) + vecX(x,3)*punto1(x,3))/(vecX(x,1)^2 + vecX(x,2)^2 + vecX(x,3)^2);
    p_sal1(x,:) = [punto1(x,1) + t(x)*vecX(x,1), punto1(x,2) + t(x)*vecX(x,2), punto1(x,3) + t(x)*vecX(x,3)];
end

%vec_sal = [punt[0] + t*vec_nom[0], punt[1] + t*vec_nom[1], punt[2] + t*vec_nom[2]]

