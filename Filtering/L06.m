%% Setting up workspace
close all; % Close all figures and windows
clear; % Clear workspace
clc; % Clears screen
%% Initializing parameters
tf = 1; % Final time in simulation
T = 1e-3; % Sample rate
t = 0:T:tf; % Time vector
n = 3; % Number of states
m = 3; % Number of measurements
A = [1 T 0; 0 1 T; -557.02 -28.616 0.9418]; % System matrix
B = [0; 0; 557.02]; % Input matrix
C = eye(m); % Measurement matrix
Q = diag([1e-5 1e-3 1e-1]); % Defines Q
R = diag([1e-4 1e-2 1]); % Defines R
w = mvnrnd(zeros(length(t),n),Q)'; % Defines w
v = mvnrnd(zeros(length(t),m),R)'; % Defines v
x = zeros(n, length(t)); % Initializes states to zero
z = zeros(m, length(t)); % Initializes measurements to zero
u = randn(1, length(t)); % Random input
u = u./max(abs(u)); % Normalizes input between -1 and +1
u(0.5/T:end) = u(0.5/T:end) + 1; % Adds step value to second-half of the input
x_kf = x;  % Initializes state estimates
P_kf = 10*Q; % Initializes state error covariance
%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    x(:,k+1) = A*x(:,k) + B*u(k) + w(:,k); % Linear state space equation
    z(:,k+1) = C*x(:,k+1) + v(:,k+1); % Linear measurement equation
    [x_kf(:,k+1), P_kf(:,:,k+1)] = kf(x_kf(:,k), z(:,k+1), u(k), P_kf(:,:,k), A, B, C, Q, R); % Calls KF function to estimate states  
    %     % KF - Prediction
    %     x_kf(:,k+1) = A*x_kf(:,k) + B*u(k);
    %     P_kf(:,:,k+1) = A*P_kf(:,:,k)*A' + Q;
    %     % KF - Update
    %     K = P_kf(:,:,k+1)*C'/(C*P_kf(:,:,k+1)*C' + R);
    %     x_kf(:,k+1) = x_kf(:,k+1) + K*(z(:,k+1) - C*x_kf(:,k+1));
    %     P_kf(:,:,k+1) = (eye(n) - K*C)*P_kf(:,:,k+1)*(eye(n) - K*C)' + K*R*K';
end
%% Results
figure; plot(t, x(1,:)); hold all; plot(t, x_kf(1,:)); xlabel('Time (sec)'); ylabel('Position'); legend('True','KF'); % Plots position and estimated position with time
% figure; plot(t, x(2,:)); xlabel('Time (sec)'); ylabel('Velocity'); % Plots velocity with time
% figure; plot(t, x(3,:)); xlabel('Time (sec)'); ylabel('Acceleration'); % Plots acceleration with time
% figure; plot(t, u); xlabel('Time (sec)'); ylabel('Input'); % Plots input with time