%% Setting up workspace
close all; % Close all figures and windows
clear; % Clear workspace
clc; % Clears screen
%% Initializing parameters
tf = 15; % Final time in simulation
T = 2e-2; % Sample rate
t = 0:T:tf; % Time vector

%% Definitions for the Wheels
n_m = 2; % Number of states
m_m = 1; % Number of measurements
p_m = 1; % Number of inputs
num_m=4; %Number of Motors
A_c = [-0.1/0.001 , 0.01/0.001; -0.01/0.2 , -1/0.2]; % System matrix for single motor in continuous time
B_c = [0;1/0.3]; % Input matrix for single motor in continuous time
C_c = [1,0]; % Measurement matrix for single motor in continuous time
A_d=A_c*T+eye(n_m); %discrete system matrix
B_d=B_c*T; %Discrete input matrix
C_d=C_c; %Discrete Measurement Matrix
A_m=blkdiag(A_d,A_d,A_d,A_d);
B_m=blkdiag(B_d,B_d,B_d,B_d);
C_m=blkdiag(C_d,C_d,C_d,C_d); %Measurement Matrix
Q_m = diag([0.009 0.08 0.009 0.08 0.009 0.08 0.009 0.08])*T*1e-3; % Defines Q
R_m = diag([0.003,0.003,0.003,0.003]); % Defines R
w_m = mvnrnd(zeros(length(t),n_m*num_m),Q_m)'*T; % Defines w
v_m = mvnrnd(zeros(length(t),m_m*num_m),R_m)'; % Defines v
x_m = zeros(n_m*num_m, length(t)); % Initializes states to zero
z_m = zeros(m_m*num_m, length(t)); % Initializes measurements to zero
u_m = randn(p_m*num_m, length(t)); % Random input
u_m = u_m./max(abs(u_m)); % Normalizes input between -1 and +1
u_m=[20*sin(t*2*pi);12*sin(t*2*pi);6*sin(t*2*pi);3*sin(t*2*pi)];%sinusoidal input
x_kf_m = x_m;  % Initializes state estimates
P_kf_m = Q_m; % Initializes state error covariance

RMSE_m = zeros(n_m*num_m,1); % Initializes RMSE vector
error_m = (x_m(:,1) - x_kf_m(:,1)).^2;

%% Definitions for the Robot
n_r = 3; % Number of states
m_r = 3; % Number of measurements
p_r = 4; % Number of Inputs
C_r = eye(m_r); % Defines measurement matrix
x_r = zeros(n_r,length(t)); % Initialize states
x_d_r = x_r; %Initializes desired x states
xdot_d_r = x_r; %initialises desired xdot states
e_r = x_r; %set up error term
z_r = zeros(m_r,length(t)); % Initialize measurements
u_r = zeros(p_r,length(t)); % Initialize input
Q_r = 4e-5*eye(n_r)*T; % Defines system noise covariance
R_r = 1e3*Q_r; % Defines measurement noise covariance
P_ekf_r = 10*Q_r; % Iniitialize EKF state error covariance
x_ekf_r = x_r; % Initialize EKF estimates
w_r = mvnrnd(zeros(n_r,1), Q_r, length(t))'; % Defines system noise (zero mean and covariance Q)
v_r = mvnrnd(zeros(m_r,1), R_r, length(t))'; % Defines measurement noise (zero mean and covariance R)
rr=0.195; %define robot radius
wr = 0.03175; %define wheel radius
xNon = @(y)[(-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))-sin(y(3)))/(2*sqrt(2));
    (cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2));
    1/(4*rr), 1/(4*rr), 1/(4*rr), 1/(4*rr)]*(wr*[y(4);y(5);y(6);y(7)]+[y(8);y(9);y(10);y(11)])*T+[y(1);y(2);y(3)];
Alin = @(y) [ 1, 0, -T*((2^(1/2)*(y(8) + y(4)*wr)*(cos(y(3)) - sin(y(3))))/4 - (2^(1/2)*(y(10) + y(6)*wr)*(cos(y(3)) - sin(y(3))))/4 - (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(9) + y(5)*wr))/4 + (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(11) + y(7)*wr))/4);
 0, 1, -T*((2^(1/2)*(y(9) + y(5)*wr)*(cos(y(3)) - sin(y(3))))/4 - (2^(1/2)*(y(11) + y(7)*wr)*(cos(y(3)) - sin(y(3))))/4 + (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(8) + y(4)*wr))/4 - (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(10) + y(6)*wr))/4);
 0, 0, 1];%linearized dynamics input at [x;u;vs]
B_r = @(y)[-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    -((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr];
vs_r = zeros(p_r,length(t)); %initialize slip values
vshat_r = vs_r; %intialized measured slip values
RMSE_r = zeros(n_r,1); % Initializes RMSE vector
RMSE = zeros(n_r,1); % Initializes RMSE vector
error_r = (x_r(:,1) - x_ekf_r(:,1)).^2; % Initializes squared error

%% set up for the controller
K= [0.2 0 0;
   0 0.2 0;
   0 0 0.2]*5e0;

lambda = [2 0 0;
          0 2 0;
          0 0 1]*1.4e1;
for k=1:length(t) %Fill in desired path
%    x_d_r(:,k)=[0;0;0]; %stabilize to the origin
%     x_d_r(:,k)=[k*T*0.5;k*T*0.5;k*T*0.5]; %follow linear trajectory
     
%      if rem(((k)*T),3)==0 %random locations generator
%          x_d_r(:,k)=-0.5+1*rand(3,1);
%      elseif k~=1
%          x_d_r(:,k)=x_d_r(:,k-1);
%      end
       
     rose = 3; %parameter to create number of rose pedals
     x_d_r(:,k)=1*[cos(rose*k*T*2*pi/30)*cos(k*T*2*pi/30); cos(rose*k*T*2*pi/30)*sin(k*T*2*pi/30);sin(k*T*2*pi)]; %create rose path
     if k~=1 
         xdot_d_r(:,k)=(x_d_r(:,k)-x_d_r(:,k-1))/T; 
     end

%      x_d_r(:,k)=0.5*[sin(k*T*2*pi/12)+cos(k*T*2*pi/14);sin(k*T*2*pi/5)+cos(k*T*2*pi/4);0]; %follow linear trajectory

      vs_r(:,k)=-0.1+0.2*rand(4,1); %set the amount of real slip
end

x_r(:,1)=[1;0;0]; %any initial condition
x_r(:,1)=x_d_r(:,1); %start at the proper position

z_r(:,1)=x_r(:,1); %set first measurement
x_ekf_r=x_r(:,1); %correct starting point for ekf

%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    %calculate the controller, (wheel voltage)
    e_r(:,k) = x_ekf_r(:,k)-x_d_r(:,k); %calculate the error
    eint = sum(e_r,2)*T; %calculate the integral error term
%     if k>2/T %calculate integral error term over previous 2 seconds
%         eint=sum(e_r(:,k-2/T:k),2);
%     end
    
    s=e_r(:,k)+lambda*eint; %calculate sliding surface
    u_m(:,k+1) = 1/wr*(B_r(x_ekf_r(:,k))*(-lambda*e_r(:,k)+xdot_d_r(:,k))-vshat_r(:,k))-B_r(x_r(:,k))*K*sign(s);
    
%     %limit maximum voltage
%     if max(abs(u_m(:,k+1)))>70
%        u_m(:,k+1)=u_m(:,k+1)/max(abs(u_m(:,k+1)))*70;
%     end
    
%    u_m(:,k+1)=120*[sin(k*T*2*pi/5);cos(k*T*2*pi/5);-sin(k*T*2*pi/5);-cos(k*T*2*pi/5)]; %force controller
    
    %Filter the motors
    x_m(:,k+1) = A_m*x_m(:,k) + B_m*u_m(:,k) + w_m(:,k); % Linear state space equation
    z_m(:,k+1) = C_m*x_m(:,k+1) + v_m(:,k+1); % Linear measurement equation
    [x_kf_m(:,k+1), P_kf_m(:,:,k+1)] = kf(x_kf_m(:,k), z_m(:,k+1), u_m(:,k), P_kf_m(:,:,k), A_m, B_m, C_m, Q_m, R_m); % Calls KF function to estimate states
    
    z_kf_m=C_m*x_kf_m(:,k);
    
    %Filter the robot
    %x_r(:,k+1) = x_r(:,k) + pinv(B_r(x_r(:,k)))*(wr*u_m(:,k+1)+vs_r(:,k))*T;
    x_r(:,k+1)= xNon([x_r(:,k);z_kf_m;vs_r(:,k)])+w_r(:,k); % Linear state space equation
    z_r(:,k+1) = C_r*x_r(:,k)+v_r(:,k); % Linear measurement equation
    [x_ekf_r(:,k+1), P_ekf_r(:,:,k+1)] = ekf(x_ekf_r(:,k), z_r(:,k+1), z_kf_m, vshat_r(:,k), P_ekf_r(:,:,k), xNon, Alin, C_r, Q_r, R_r); % Calls EKF function to estimate states  
    
    
    error_r(:,k+1) = (x_r(:,k+1) - x_ekf_r(:,k+1)).^2;
    error_m(:,k+1) = (x_m(:,k+1) - x_kf_m(:,k+1)).^2;
end

for k = 1:n_m*num_m
    RMSE_m(k) = (sum(error_m(k,:))/length(t))^0.5; % Calculates the RMSE for EKF using square error values calculated in the for loop
end
for k = 1:n_r
    RMSE_r(k) = (sum(error_r(k,:))/length(t))^0.5; % Calculates the RMSE for EKF using square error values calculated in the for loop
end
fprintf('RMSE (KF): %s\n', RMSE_m)
fprintf('RMSE (EKF): %s\n', RMSE_r)
%% Results
figure; plot(t, x_m(3,:)); hold all; plot(t, x_kf_m(3,:)); xlabel('Time (sec)'); ylabel('Angular Velocity'); legend('True','KF');hold off; % Plots position and estimated position with time
figure; plot(t, x_m(2,:)); hold all; plot(t, x_kf_m(2,:));xlabel('Time (sec)'); ylabel('Velocity'); legend('True','KF');hold off; % Plots velocity with time
% figure; plot(t, u); xlabel('Time (sec)'); ylabel('Input'); % Plots input with time

figure; plot(t, x_r(1,:)); hold all; plot(t, x_ekf_r(1,:));plot(t, x_d_r(1,:)); xlabel('Time (sec)'); ylabel('x position (m)'); legend('True','KF','desired');hold off; % Plots position and estimated position with time
figure; plot(x_ekf_r(1,:), x_ekf_r(2,:)); hold all; plot(x_r(1,:), x_r(2,:));plot(x_d_r(1,:), x_d_r(2,:)); xlabel('x'); ylabel('y'); legend('EKF','True','Desired');hold off; % Plots position and estimated position with time

