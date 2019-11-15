%% Question 3
% % 
clc
clear all

dt=0.01;
t_final=10;

outputs = zeros(t_final/dt,5);

theta1=0;theta2=0;dtheta1=0;dtheta2=0; %declare variables
y=[0;0];yd=[0;0];dyd=[0;0];ddyd=[0;0]; %will redefine these
m1=10;m2=10;L1=1;L2=1; %declare mass and lengths
r1=L1/2;r2=L2/2; I1=1/12*m1*L1^2;I2=1/12*m2*L2^2; %calculate r and I

alpha = I1+I2+m1*r1^2+m2*(L1^2+r2^2);
beta = m2*L1*r2;
delta = I2+m2*r2^2;

z2=[dtheta1;dtheta2];
z1=[theta1;theta2];

A=[alpha+2*beta*cos(z1(2)) , delta+beta*cos(z1(2)) ; delta+beta*cos(z1(2)) , delta];
B=[-beta*sin(z2(2)), -beta*sin(z1(2))*(z2(1)+z1(2));beta*sin(z1(2))*z2(1),0];
C=[1,0;0,1];

k1=[-40,0;0,-40]; % THESE BETTER HAVE NEGATIVE EIGSSSS
k2=[-500,0;0,-500];

u=-k1*(dyd-z2)-k2*(yd-z1);
tau=A*(ddyd-u)+B*z1;


%x_k+1=(Ax+Bu)*dt+x
%y_next = Cx*dt+y

%create some random points
randominput = rand(2,20)*6-3;

for t=1:t_final/dt
    
    %sinusoidal functions
    yd = [sin(t*dt);sin(t*dt)];
    dyd = [cos(t*dt);cos(t*dt)];
    ddyd = [-sin(t*dt);-sin(t*dt)];
    
%     %step functions
%     yd = [square(t*dt/2);-square((t*dt+2)/2)];
%     dyd=[0;0];
%     ddyd=[0;0];
    
%     %random inputs
%     yd = randominput(:,int8(t*dt/3)+1);
%     dyd=[0;0];
%     ddyd=[0;0];
    
    
    
    y=C*z1;
    u=-k1*(dyd-z2)-k2*(yd-z1);
    tau=A*(ddyd+u)+B*z1; %WATCH IT U SILLY SILLY MAN
    
    z1=z2*dt+z1;
    z2=(A)^(-1)*(tau-B*z2)*dt+z2;

    outputs(int32(t),:)=[t*dt,yd(1),yd(2),y(1),y(2)];
end

plot(outputs(:,1),outputs(:,2),outputs(:,1),outputs(:,4),outputs(:,1),outputs(:,3),outputs(:,1),outputs(:,5));
legend('\theta_{1\_desired}','\theta_{1\_real}','\theta_{2\_desired}','\theta_{2\_real}')


