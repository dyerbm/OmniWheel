%% Setting up workspace
close all; % Close all figures and windows
clear; % Clear workspace
clc; % Clears screen
%% Initializing parameters
tf = 1; % Final time in simulation
T = 2e-5; % Sample rate
t = 0:T:tf; % Time vector

n_m = 2; % Number of states
m_m = 1; % Number of measurements
p_m = 1; % Number of inputs
num_m=4; %Number of Motors
J = 0.01;
b = 1;
K_m = 1;
R = 0.6;
L = 0.01;
A_c=[-b/J K_m/J;-K_m/L -R/L];
B_c=[0; 1/L];
C_c=[1 0];
A_d=A_c*T+eye(n_m); %discrete system matrix
B_d=B_c*T; %Discrete input matrix
C_d=C_c; %Discrete Measurement Matrix
A_m=blkdiag(A_d,A_d,A_d,A_d);
B_m=blkdiag(B_d,B_d,B_d,B_d);
C_m=blkdiag(C_d,C_d,C_d,C_d); %Measurement Matrix

x_m = zeros(n_m*num_m, length(t)); % Initializes states to zero
z_m = zeros(m_m*num_m, length(t)); % Initializes measurements to zero
u_m = randn(p_m*num_m, length(t)); % Random input
e_m= z_m;
e_d_m=0;
eint_m=0;

K_p=-100;
K_d=-1;
K_i=-100;

%% Set initial conditions

for k=1:length(t) %Fill in desired path
     x_d(:,k)=[1;1;1;1]; %stabilize to the origin
end

x(:,1)=[0;0;0;0]; %any initial condition
%x(:,1)=x_d(:,1); %start at the proper position

z(:,1)=x(:,1); %set first measurement


%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    
    
    e_m(:,k)=z_m(:,k)-x_d(:,k);
    eint_m=sum(e_m,2)*T;
    if k>2/T %calculate integral error term over previous 2 seconds
        eint_m=sum(e_m(:,k-2/T:k),2);
    end
    if k~=1
       e_d_m = (e_m(:,k)-e_m(:,k-1))/T;
    end
    u_m(:,k+1)=K_p*e_m(:,k)+K_d*e_d_m+K_i*eint_m;%calculate voltages for each motor
    if abs(max(u_m(:,k+1)))>12
       u_m(:,k+1)= u_m(:,k+1)/abs(max(u_m(:,k+1)))*12;
       k
    end
    
    %u_m(:,k+1)=[12;12;12;12];%force linear controller
    
    u_m(:,k+1);
    
    x_m(:,k+1)=A_m*x_m(:,k)+B_m*u_m(:,k+1);
    z_m(:,k+1)=C_m*x_m(:,k+1);
    
    x_m(:,k+1);
end


figure; plot(t, z_m(1,:));hold on;plot(t, x_d(1,:)); xlabel('Time (sec)');hold off;legend('real','desired') % Plots position with time
figure; plot(t, z_m(2,:));hold on;plot(t, x_d(2,:)); xlabel('Time (sec)');hold off;legend('real','desired') % Plots velocity with time
figure; plot(t, z_m(3,:));hold on;plot(t, x_d(3,:)); xlabel('Time (sec)');hold off;legend('real','desired') % Plots acceleration with time


% J = 0.0001;
% b = 0.001;
% K = 105;
% R = 10;
% L = 0.01;
% s = tf('s');
% P_motor = K/((J*s+b)*(L*s+R)+K^2);
% 
% Kp = 100;
% Ki = 200;
% Kd = 10;
% C = pid(Kp,Ki,Kd);
% sys_cl = feedback(C*P_motor,1);
% step(sys_cl, 0:0.01:4)
% grid
% title('PID Control with Large Ki and Large Kd')