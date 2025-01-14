clc
clear all
syms the1 the2 the3 L1 L2 L3 r11 r12 r13 r21 r22 r23 r31 r32 r33 Px Py Pz d
%% FK
T01=FK(0,0,d,0)
T12=FK(0,L1,0,the1)
T23=FK(0,L2,0,the2)
T02=simplify(T01*T12)
T03=simplify(T01*T12*T23)

%% IK
T03_IK=[r11 r12 r13 Px;r21 r22 r23 Py;r31 r32 r33 Pz;0 0 0 1]
T13i=simplify(inv(T01)*T03_IK)
T13f=simplify(T12*T23)


