%% Question 3
% % 
clc
clear all

dt=0.02;
t_final=20;
t=0:dt:t_final;

n=3; %num states
m=3;%num measurements

rR=0.195; %COM to wheel distance
M=3.4; %mass of the robot
IR=0.069; %robot moment of inertia

A=[-3.648 0 0; 0 -3.653 0; 0 0 -193];
B=[-0.0024/M 0 0.00124/M 0;
    0 0.0018/M 0 -0.0018/M;
    rR/IR rR/IR rR/IR rR/IR];
C=eye(3);

x=zeros(n,length(t));
xdot=zeros(n,length(t));
xddot=zeros(n,length(t));
z=zeros(n,length(t));

Kp=[-4.1,0,0;0,-4.1,0;0,0,-4.1;0,0,0]; % THESE BETTER HAVE NEGATIVE EIGSSSS
Kd=[-5,0,0;0,-5,0;0,0,-5;0,0,0];

%create some random points
randominput = rand(3,20)*6-3;

for k=1:length(t)-1
    %simulate system
    %xd = randominput(:,int8(k*dt/5)+1);
    xd = [5;5;0];
    dxdot=[0;0;0];
    dxddot=[0;0;0];
    
    u=Kp*(dxdot-xdot(:,k))+Kd*(xd-x(:,k)) %calculate controller based on position/velocity
    if(max(abs(u))>255) %normalize controller
        u=u/max(abs(u))*255;
    end
    

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

