%% Setting up workspace
close all; % Close all figures and windows
clear; % Clear workspace
clc; % Clears screen
%% Initializing parameters
tf = 1; % Final time in simulation
T = 1e-3; % Sample rate
t = 0:T:tf; % Time vector
n = 2; % Number of states
m = 2; % Number of measurements
zeta = 0.2; % Defines zeta
omega = 5; % Defines omega
A = [1 T; -omega^2*T (1-2*zeta*omega*T)]; % System matrix
B = [0; 12]; % Input matrix
C = eye(m); % Measurement matrix
x = zeros(n, length(t)); % Initializes states to zero
z = zeros(m, length(t)); % Initializes measurements to zero
Q = [1e-6 2e-6; 2e-6 5e-3]; % Defines system noise covariance
R = diag([0.01 0.01]); % Defines measurement noise covariance
w = mvnrnd(zeros(length(t),n),Q)'; % Defines system noise
v = mvnrnd(zeros(length(t),m),R)'; % Defines measurement noise
%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    x(:,k+1) = A*x(:,k) + B*T + [0; T].*w(:,k); % Linear state space equation
    z(:,k+1) = C*x(:,k+1) + v(:,k+1); % Linear measurement equation
end
%% Results
figure; plot(t, x(1,:), 'LineWidth', 2); hold all; plot(t, z(1,:));
legend('State 1', 'Measurement 1');
xlabel('Time (sec)'); ylabel('Position (m)'); % Plots position with time
figure; plot(t, x(2,:), 'LineWidth', 2); hold all; plot(t, z(2,:));
legend('State 2', 'Measurement 2');
xlabel('Time (sec)'); ylabel('Velocity (m/s)'); % Plots velocity with time