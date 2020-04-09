%% Setting up workspace
close all; % Close all figures and windows
clear; % Clear workspace
clc; % Clears screen
%% Initializing parameters
tf = 10; % Final time in simulation
T = 2e-2; % Sample rate
t = 0:T:tf; % Time vector

%Definitions for the Wheels
n_m = 2; % Number of states
m_m = 1; % Number of measurements
p_m = 1; % Number of inputs
num_m=4; %Number of Motors
A_c = [-0.1/0.001 , 0.01/0.001; -0.01/0.5 , -1/0.5]; % System matrix for single motor in continuous time
B_c = [0;1/0.5]; % Input matrix for single motor in continuous time
C_c = [1,0]; % Measurement matrix for single motor in continuous time
A_d=A_c*T+eye(n_m); %discrete system matrix
B_d=B_c*T; %Discrete input matrix
C_d=C_c; %Discrete Measurement Matrix
A_m=blkdiag(A_d,A_d,A_d,A_d);
B_m=blkdiag(B_d,B_d,B_d,B_d);
C_m=blkdiag(C_d,C_d,C_d,C_d); %Measurement Matrix
Q_m = diag([0.009 0.08 0.009 0.08 0.009 0.08 0.009 0.08])*T; % Defines Q
R_m = diag([0.03,0.03,0.03,0.03]); % Defines R
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

%Definitions for the Robot
n_r = 3; % Number of states
m_r = 3; % Number of measurements
C_r = eye(m); % Defines measurement matrix
x_r = zeros(n,length(t)); % Initialize states
z_r = zeros(m,length(t)); % Initialize measurements
u_r = zeros(1,length(t)); % Initialize input

Q_r = 1e-9*eye(n); % Defines system noise covariance
R_r = 1e3*Q; % Defines measurement noise covariance
P_ekf_r = 10*Q; % Iniitialize EKF state error covariance
x_ekf_r = x; % Initialize EKF estimates
w_r = mvnrnd(zeros(n,1), Q, length(t))'; % Defines system noise (zero mean and covariance Q)
v_r = mvnrnd(zeros(m,1), R, length(t))'; % Defines measurement noise (zero mean and covariance R)
xNon = @(y)[(-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))-sin(y(3)))/(2*sqrt(2));
    (cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2));
    1/(4*rr), 1/(4*rr), 1/(4*rr), 1/(4*rr)]*(wr*[y(4);y(5);y(6);y(7)]+[y(8);y(9);y(10);y(11)])*T+[y(1);y(2);y(3)];
Alin = @(y) [ 1, 0, -T*((2^(1/2)*(y(8) + y(4)*wr)*(cos(y(3)) - sin(y(3))))/4 - (2^(1/2)*(y(10) + y(6)*wr)*(cos(y(3)) - sin(y(3))))/4 - (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(9) + y(5)*wr))/4 + (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(11) + y(7)*wr))/4);
 0, 1, -T*((2^(1/2)*(y(9) + y(5)*wr)*(cos(y(3)) - sin(y(3))))/4 - (2^(1/2)*(y(11) + y(7)*wr)*(cos(y(3)) - sin(y(3))))/4 + (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(8) + y(4)*wr))/4 - (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(10) + y(6)*wr))/4);
 0, 0, 1];%linearized dynamics input at [x;u;vs]
RMSE = zeros(n,1); % Initializes RMSE vector
error = (x(:,1) - x_ekf(:,1)).^2; % Initializes squared error


%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    %calculate the controller
    
    %Filter the motors
    x_m(:,k+1) = A_m*x_m(:,k) + B_m*u_m(:,k) + w_m(:,k); % Linear state space equation
    z_m(:,k+1) = C_m*x_m(:,k+1) + v_m(:,k+1); % Linear measurement equation
    [x_kf_m(:,k+1), P_kf_m(:,:,k+1)] = kf(x_kf_m(:,k), z_m(:,k+1), u_m(:,k), P_kf_m(:,:,k), A_m, B_m, C_m, Q_m, R_m); % Calls KF function to estimate states
    
    %Filter the robot
    x_r(:,k+1) = x_r(:,k) + Binv(x(:,k))*(wr*u_r(:,k+1)+vs_r(:,k))*T; % Linear state space equation
    z_r(:,k+1) = C_r*x_r(:,k+1); % Linear measurement equation

    error_m(:,k+1) = (x_m(:,k+1) - x_kf_m(:,k+1)).^2;
end

for k = 1:n_m*num_m
    RMSE_m(k) = (sum(error_m(k,:))/length(t))^0.5; % Calculates the RMSE for EKF using square error values calculated in the for loop
end

fprintf('RMSE (KF): %s\n', RMSE_m)
%% Results
figure; plot(t, x_m(3,:)); hold all; plot(t, x_kf_m(3,:)); xlabel('Time (sec)'); ylabel('Angular Velocity'); legend('True','KF');hold off; % Plots position and estimated position with time
figure; plot(t, x_m(2,:)); hold all; plot(t, x_kf_m(2,:));xlabel('Time (sec)'); ylabel('Velocity'); legend('True','KF');hold off; % Plots velocity with time
% figure; plot(t, u); xlabel('Time (sec)'); ylabel('Input'); % Plots input with time