syms theta1 theta2 theta3 L1 L2 L3 d_theta1 d_theta2 d_theta3 d
assume(L1,'real');
assume(L2,'real');
assume(L3,'real');
assume(theta1,'real');
assume(theta2,'real');
assume(theta3,'real');
assume(d_theta1,'real');
assume(d_theta2,'real');
assume(d_theta3,'real');

%FK:
[T01,R01,R10,P01] = FK(0,0,d,0);
[T12,R12,R21,P12] = FK(0,L1,0,theta1);
[T23,R23,R32,P23] = FK(0,L2,0,theta2);
T03 = T01*T12*T23
PEE = ENDEFFECTOR(d,theta1,theta2,L1,L2,L3);

%IK:
%declare the parameter:
L1=1; L2=1; L3=1;
x=1;
y=0;
z=1;
fprintf("Initial value: x = %.4f, y = %.4f,z = %.4f \n\n", x,y,z);
fprintf("--------------------------------------------------------------------------------\n");

%% theta2:
c2=((x-L1)^2+y^2-L3^2-L1^2)/(2*L2*L3);
% Bo nghiem 1
for i=1:2
    if i==1
      s2=sqrt(1-c2^2);
    else
      s2=-sqrt(1-c2^2);
    end
theta2=atan2(s2,c2);
c1=((x-L1)*(L3*cos(theta2)+L2)+y*L3*sin(theta2))/((L3*cos(theta2)+L2)*(L3*cos(theta2)+L2)+(L3*sin(theta2))^2);
s1=((L3*cos(theta2)+L2)*y -(x-L1)*L3*sin(theta2))/((L3*cos(theta2)+L2)*(L3*cos(theta2)+L2)+(L3*sin(theta2))^2);
fprintf('Bo nghiem: %d ',i);
theta2
theta1=atan2(s1,c1)
d=z 
%% draw robot
subplot(1,2,i);
PEE = ENDEFFECTOR(d,theta1,theta2,L1,L2,L3);
plot3([0;0], [0;0],[0;d],'-ko',...
      [0;L1],[0,0],[d;d],'-ko',...
      [L1;L1+L2*cos(theta1)],[0;L2*sin(theta1)],[d;d],'-ko',...
      [L1+L2*cos(theta1);PEE(1)],[L2*sin(theta1);PEE(2)],[d;PEE(3)],'-ko','linewidth',3);
  title(' VI TRI ROBOT ');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
xlim([-5 5]);
ylim([-5 5]);
zlim([-5 5]);
end
disp(' Vay co 2 nghiem Phan Biet ');



 
