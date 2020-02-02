%% Question 3
% % 
clc
clear all

dt=0.02;
t_final=10;
t=0:dt:t_final;

n=3; %num states
m=3;%num measurements

rR=0.195; %COM to wheel distance
M=3.4; %mass of the robot
IR=0.056282655290730/255; %robot moment of inertia

A=[-1.391610614393775 0 0; 0 -1.238717062996583 0; 0 0 -12.0207583];
B=[2.3884e-5/M 0 -2.3884e-5/M 0;
    0 1.7482765e-5/M 0 -1.7482765e-5/M;
    2e-5*8.835e2 2e-5*8.835e2 2e-5*8.835e2 2e-5*8.835e2]*1000;
C=eye(3);

x=zeros(n,length(t));
xdot=zeros(n,length(t));
xddot=zeros(n,length(t));
z=zeros(n,length(t));

Kp=[1000,0,-30;0,1000,30;-1000,0,30;0,-1000,-30]; % THESE BETTER HAVE NEGATIVE EIGSSSS
Kd=[500,0,0;0,500,0;-500,0,0;0,-500,0];

%create some random points
randominput = rand(3,20)*6-3;

for k=1:length(t)-1
    %simulate system
    %xd = randominput(:,int8(k*dt/5)+1);
    dx = [1;1;3];
    dxdot=[0;0;0];
    dxddot=[0;0;0];
    
    x(:,k)
    u=Kd*(dxdot-xdot(:,k))+Kp*(dx-x(:,k)); %calculate controller based on position/velocity
    if(max(abs(u))>255) %normalize controller
        u=u/max(abs(u))*255
    end
    %u=[-255;255;255;-255];
    

    xddot(:,k+1)=A*xdot(:,k)+B*u; %calculate new acceleration
    xdot(:,k+1) = xdot(:,k)+xddot(:,k)*dt; %calculate new velocity
    x(:,k+1) = x(:,k)+xdot(:,k+1)*dt; %calculate new position
    
    
    z(:,k+1)=C*xdot(:,k+1); %measure the velocity
    
end

figure
plot(t,x(1,:),t,x(2,:));
legend('x','y')
xlabel('time (s)')
ylabel('position (m,rad)')
