%% Question 3
% % 
clc
clear all

dt=0.02;
t_final=10000;
t=0:dt:t_final;

n=3; %num states
m=3;%num measurements
p=4;%num inputs

rR=0.195; %COM to wheel distance
M=3.4; %mass of the robot
IR=0.056282655290730/255; %robot moment of inertia

A=[-1.391610614393775 0 0; 0 -1.238717062996583 0; 0 0 -12.0207583];
B=[0 -2.3884e-5/M 0 2.3884e-5/M;
     1.7482765e-5/M 0 -1.7482765e-5/M 0;
    2e-5*8.835e2 2e-5*8.835e2 2e-5*8.835e2 2e-5*8.835e2];
% B=[0 -2e-5/M 0 2e-5/M;
%      2e-5/M 0 -2e-5/M 0;
%     2e-5*8.835e2 2e-5*8.835e2 2e-5*8.835e2 2e-5*8.835e2]*10000
C=eye(3);

x=zeros(n,length(t));
xdot=zeros(n,length(t));
xddot=zeros(n,length(t));
z=zeros(n,length(t));

u=zeros(p,length(t));

scale=0.8;
% Kp=[600,0,20;
%     0,600,20;
%     -600,0,20;
%     0,-600,20]*scale; % THESE BETTER HAVE NEGATIVE EIGSSSS
% Kd=[500,0,2;
%     0,500,2;
%     -500,0,2;
%     0,-500,2]*scale*2;
scale=1000;
Kp=[6,0,1e-1;
    0,6,1e-1;
    -6,0,1e-1;
    0,-6,1e-1]*scale; % THESE BETTER HAVE NEGATIVE EIGSSSS
Kd=[5,0,-1e-3;
    0,5,-1e-3;
    -5,0,-1e-3;
    0,-5,-1e-3]*scale*2;


%create some random points
randominput = rand(3,20)*6-3;


x(:,1)=[0; 0; 0];
for k=1:length(t)-1
    %simulate system
%     dx = randominput(:,int8(k*dt/5)+1);
%     dx(3)=0;
    dx = [1;0;0];
    dxdot=[0;0;0];
    dxddot=[0;0;0];
    
    
    RotationMatrix = [cos(x(3,k)) -sin(x(3,k)) 0; sin(x(3,k)) cos(x(3,k)) 0; 0 0 1];
    robotXOrientation = RotationMatrix*[cos(0); sin(0); 1];
    robotYOrientation = RotationMatrix*[cos(2*pi/4); sin(2*pi/4); 1];
    robotTOrientation = [0;0;1];
    dxr = [dot(dx,robotXOrientation)/norm(robotXOrientation)
        dot(dx,robotYOrientation)/norm(robotYOrientation)
        dot(dx,robotTOrientation)/norm(robotTOrientation)];
    dxdotr = [dot(dxdot,robotXOrientation)/norm(robotXOrientation)
        dot(dxdot,robotYOrientation)/norm(robotYOrientation)
        dot(dxdot,robotTOrientation)/norm(robotTOrientation)];
    dxddotr = [dot(dxddot,robotXOrientation)/norm(robotXOrientation)
        dot(dxddot,robotYOrientation)/norm(robotYOrientation)
        dot(dxddot,robotTOrientation)/norm(robotTOrientation)];
    
    xr = [dot(x(:,k),robotXOrientation)/norm(robotXOrientation)
        dot(x(:,k),robotYOrientation)/norm(robotYOrientation)
        dot(x(:,k),robotTOrientation)/norm(robotTOrientation)];
    xdotr = [dot(xdot(:,k),robotXOrientation)/norm(robotXOrientation)
        dot(xdot(:,k),robotYOrientation)/norm(robotYOrientation)
        dot(xdot(:,k),robotTOrientation)/norm(robotTOrientation)];
    
    dxr-xr;
    u(:,k)=(Kd*(dxdotr-xdotr)+Kp*(dxr-xr)); %calculate controller based on position/velocity
    %u=(Kd*(dxdot-xdot(:,k))+Kp*(dx-x(:,k)));
    if(max(abs(u(:,k)))>255) %normalize controller
        u(:,k)=u(:,k)/max(abs(u(:,k)))*255;
        u(:,k)
        k
    end
    %u=[-255;255;255;-255];
    

    xddot(:,k+1)=A*xdot(:,k)+B*u(:,k); %calculate new acceleration
    xdot(:,k+1) = xdot(:,k)+xddot(:,k)*dt; %calculate new velocity
    x(:,k+1) = x(:,k)+xdot(:,k+1)*dt; %calculate new position
    
    
    z(:,k+1)=C*xdot(:,k+1); %measure the velocity
    
end

figure
plot(t,x(1,:),t,x(2,:),t,x(3,:));
legend('x','y','\theta')
xlabel('time (s)')
ylabel('position (m,rad)')

