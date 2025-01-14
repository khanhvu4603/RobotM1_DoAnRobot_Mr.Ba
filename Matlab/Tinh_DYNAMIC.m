syms theta1 theta2 theta3 L1 L2 L3 d_theta1 d_theta2 d dd

P1c = [L1; 0; 0; 1];
P2c = [L2; 0; 0; 1];
P3c = [L3; 0; 0; 1];
P1c3x = [L1; 0; 0];
P2c3x = [L2; 0; 0];
P3c3x = [L3; 0; 0];
w0 = [0; 0; 0];
v0 = [0; 0; 0];

%Link1:
TP01c = (T01*P1c);
P01c = TP01c(1:3);
%Vận tốc góc:
w1 = R10*w0;
%Vận tốc dài:
v1 = R10*(v0+[0 -w0(3) w0(2); w0(3) 0 -w0(1); -w0(2) w0(1) 0]*P01)+dd*[0 0 1]';
%Vận tốc dài trọng tâm:
v1c = R01*(v1 + [0 -w1(3) w1(2); w1(3) 0 -w1(1); -w1(2) w1(1) 0]*P1c3x);

%Link2:
TP02c = (T01*T12*P2c);
P02c = TP02c(1:3);
%Vận tốc góc:
w2 = R21*w1+d_theta1*[0 0 1]';
%Vận tốc dài:
v2 = R21*(v1+[0 -w1(3) w1(2); w1(3) 0 -w1(1); -w1(2) w1(1) 0]*P12);
%Vận tốc dài trọng tâm:
v2c = R01*R12*(v2 + [0 -w2(3) w2(2); w2(3) 0 -w2(1); -w2(2) w2(1) 0]*P2c3x);

%Link3:
TP03c = (T01*T12*T23*P3c);
P03c = TP03c(1:3);
%Vận tốc góc:
w3 = R32*w2+d_theta2*[0 0 1]';
%Vận tốc dài:
v3 = R32*(v2+[0 -w2(3) w2(2); w2(3) 0 -w2(1); -w2(2) w2(1) 0]*P23);
%Vận tốc dài trọng tâm:
v3c = R01*R12*R23*(v3 + [0 -w3(3) w3(2); w3(3) 0 -w3(1); -w3(2) w3(1) 0]*P3c3x);
v3c = simplify(v3c);


syms m1 m2 m3 g dd_theta1 dd_theta2 ddd
%Động năng của hệ:
% K = 1/2*(m1*(v1c)'*v1c) + 1/2*(m2*(v2c)'*v2c) + 1/2*(m3*(v3c)'*v3c);
% K = simplify(K);
K = simplify(1/2*(m1*dd*dd)+1/2*m2*(d_theta1*d_theta1*L2*L2+dd*dd)+1/2*m3*(d_theta1*d_theta1*L2*L2+(L3*(d_theta1+d_theta2)*(d_theta1+d_theta2)+dd*dd+2*L2*L3*d_theta1*(d_theta1+d_theta2)*cos(theta2))-(m1+m2+m3)*g*d));
U = (m1+m2+m3)*g*d;
L = simplify(K - U);

dLdd = diff(L,dd);
dLd_theta1 = diff(L,d_theta1);
dLd_theta2 = diff(L,d_theta2);
dLd = diff(L,d);
dLtheta1 = diff(L,theta1);
dLtheta2 = diff(L,theta2);


t1 = (ddd+g)*(m1+m2+m3); %simplify(dLdd - dLd);
t2 = dd_theta1*(m2*L2^2+m3*L2^2+m3*L3^2+2*L2*L3*m3*cos(theta2))+dd_theta2*(m3*L3^2+L2*L3*m3*cos(theta2))-d_theta2*(L2*L3*m3*sin(theta2)*(2*d_theta1+theta2)); %simplify(dLd_theta1 - dLtheta1);
t3 = L3^2*m3*dd_theta2+dd_theta1*(m3*L2*L3*cos(theta2)+L3^2*m3)+L3*m3*L2*sin(theta2)*theta1^2; %simplify(dLd_theta2 - dLtheta2);


%Động lực học Robot

M = simplify([diff(t1,ddd),diff(t1,dd_theta1),diff(t1,dd_theta2); ...
              diff(t2,ddd),diff(t2,dd_theta1),diff(t2,dd_theta2); ...
              diff(t3,ddd),diff(t3,dd_theta1),diff(t3,dd_theta2)])

G = simplify([diff(U,d); diff(U,theta1); diff(U,theta2)])
dd_theta = [ddd;
            dd_theta1;
            dd_theta2]
Mtheta = M*dd_theta;
t = [t1;t2;t3]
V = simplify(t - Mtheta - G)
Ktra = simplify(t - Mtheta - G - V)