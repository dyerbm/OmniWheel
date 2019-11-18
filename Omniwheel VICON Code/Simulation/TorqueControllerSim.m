%% Question 3
% % 
clc
clear all

dt=0.01;
t_final=10;

outputs = zeros(t_final/dt,7);

y=[0;0;0];yd=[0;0;0];dyd=[0;0;0];ddyd=[0;0;0]; %will redefine these

z2=[0;0;0]; %velocities
z1=[0;0;0]; %position

Iw=0.00005; %wheel moment of inertia - FIX THIS VALUE
rw=0.034925; %wheel radius
rR=0.195; %COM to wheel distance
m=3.5; %mass of the robot
IR=0.05; %robot moment of inertia


I=eye(4);
S=[0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0];
M=[m,0,0;0,m,0;0,0,IR];
Q=[-sin(z1(3)), -sin(z1(3)+pi/2), -sin(z1(3)+pi), -sin(z1(3)+pi/3);
    cos(z1(3)), cos(z1(3)+pi/2), cos(z1(3)+pi), cos(z1(3)+pi/3);
    rR,rR,rR,rR];
R=[0,1,rw;-1,0,rw;0,-1,rw;1,0,rw];
T=[cos(z1(3)),sin(z1(3)),0;-sin(z1(3)),cos(z1(3)),0;0,0,1];
Tdot=z2(3)*[-sin(z1(3)),cos(z1(3)),0;-cos(z1(3)),-sin(z1(3)),0;0,0,0];

Nstar=Iw/rw*(S+I)*R*Tdot;
Mstar=rw*Q\M+Iw/rw*(S+I)*R*T;

k1=[-40,0,0;0,-40,0;0,0,-40]; % THESE BETTER HAVE NEGATIVE EIGSSSS
k2=[-500,0,0;0,-500,0;0,0,-500];

u=-k1*(dyd-z2)-k2*(yd-z1);
tau=Mstar*(ddyd+u)+Nstar*z1;


%x_k+1=(Ax+Bu)*dt+x
%y_next = Cx*dt+y

%create some random points
randominput = rand(2,20)*6-3;

for t=1:t_final/dt
    
%     %sinusoidal functions
%    yd = [0.4*sin(t*dt);0.4*sin(t*dt);0];
%    dyd = [0.4*cos(t*dt);0.4*cos(t*dt);0];
%    ddyd = [-0.4*sin(t*dt);-0.4*sin(t*dt);0];

    %go to a point
    yd=[1;0;0];
    dyd=[0;0;0];
    ddyd=[0;0;0];
    
     %step functions
%      yd = [square(t*dt/2);0;5];
%      dyd=[0;0;0];
%      ddyd=[0;0;0];
    
%     %random inputs
%     yd = randominput(:,int8(t*dt/3)+1);
%     dyd=[0;0];
%     ddyd=[0;0];

    Q=[-sin(z1(3)), -sin(z1(3)+pi/2), -sin(z1(3)+pi), -sin(z1(3)+pi/3);
    cos(z1(3)), cos(z1(3)+pi/2), cos(z1(3)+pi), cos(z1(3)+pi/3);
    rR,rR,rR,rR];
    T=[cos(z1(3)),sin(z1(3)),0;-sin(z1(3)),cos(z1(3)),0;0,0,1];
    Tdot=z2(3)*[-sin(z1(3)),cos(z1(3)),0;-cos(z1(3)),-sin(z1(3)),0;0,0,0];
    
    y=z1;
    u=-k1*(dyd-z2)-k2*(yd-z1);
    tau=Mstar*(ddyd+u)+Nstar*z1; %WATCH IT U SILLY SILLY MAN
    
    if max(abs(tau))>1.73 %max torque of 75
        tau = tau./max(abs(tau))*1.73;
    end
    
    z1=z2*dt+z1;
    z2=Mstar\(tau-Nstar*z2)*dt+z2;

    outputs(int32(t),:)=[t*dt,yd(1),yd(2),yd(3),y(1),y(2),y(3)];
end

plot(outputs(:,1),outputs(:,2),outputs(:,1),outputs(:,5),outputs(:,1),outputs(:,3),outputs(:,1),outputs(:,6),outputs(:,1),outputs(:,4),outputs(:,1),outputs(:,7));
legend('x_{desired}','x_{real}','y_{desired}','y_{real}','theta_{desired}','theta_{real}')


