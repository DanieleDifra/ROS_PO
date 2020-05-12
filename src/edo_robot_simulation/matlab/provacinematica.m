%% Inizializzaione
clear;
clc;
dh=[0.337,0,-pi/2; 0,0.21,0; 0,0,-pi/2; 0.268,0,pi/2; 0,0,-pi/2; 0.174,0,0];

cartPos = zeros(6,1);
cartVel = zeros(6,1);
A=zeros(4,4,6);
%jointPos = zeros(1,6);
jointPos = [0,-pi/2,-pi/2,-pi,-pi,0];
jointVel = zeros(6,1);
%% Esecuzione

for i=1:6
 A(:,:,i) = [ cos(jointPos(i)),-sin(jointPos(i))*cos(dh(i,3)), sin(jointPos(i))*sin(dh(i,3)),dh(i,2)*cos(jointPos(i));
              sin(jointPos(i)), cos(jointPos(i))*cos(dh(i,3)),-cos(jointPos(i))*sin(dh(i,3)),dh(i,2)*sin(jointPos(i));
              0,                        sin(dh(i,3)),               cos(dh(i,3)),                   dh(i,1);
              0,                            0,                           0,                            1];
end
T = A(:,:,1);
for i=2:6
    T(:,:,i) = T(:,:,i-1)*A(:,:,i);
end
cartPos(1:3) = T(1:3,4,6);
cartPos(4:6) = rotm2eul(T(1:3,1:3,6));

J = zeros(6,6);
J(1:3,1) = cross([0,0,1],T(1:3,4,6));
J(4:6,1) = [0;0;1];
for i = 2:6
    J(1:3,i) = cross(T(1:3,3,i-1),T(1:3,4,6)-T(1:3,4,i-1));
    J(4:6,i) = T(1:3,3,i-1);
end

R = [0,-sin(cartPos(4)),cos(cartPos(4))*sin(cartPos(5));
     0,cos(cartPos(4)), sin(cartPos(4))*sin(cartPos(5));
     1,     0,              cos(cartPos(5))];
 
cartVel = J*jointVel;
cartVel(4:6)=R*cartVel(4:6);












