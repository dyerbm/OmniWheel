%% Setting up workspace
close all; % Close all figures and windows
clear; % Clear workspace
clc; % Clears screen
%% Initializing parameters
tf = 10; % Final time in simulation
T = 1e-3; % Sample rate
t = 0:T:tf; % Time vector
n = 2; % Number of states
m = 2; % Number of measurements
A = [-0.1/0.001 , 0.01/0.001; -0.01/0.5 , -1/0.5]; % System matrix for single motor in continuous time
B = [0;1/0.5]; % Input matrix for single motor in continuous time
C_c = [1,0]; % Measurement matrix for single motor in continuous time


% A= exp(A_c*T); %Discrete system matrix for single motor
% B=inv(A_c)*(A-eye(2))*B_c;
C = eye(m); % Measurement matrix
x = zeros(n, length(t)); % Initializes states to zero
z = zeros(m, length(t)); % Initializes measurements to zero
u = randn(1, length(t)); % Random input
u = u./max(abs(u)); % Normalizes input between -1 and +1
u(0.5/T:end) = u(0.5/T:end) + 1; % Adds step value to second-half of the input
%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    x(:,k+1) = (A*x(:,k) + B*u(k))*T+x(:,k); % Linear state space equation
    z(:,k+1) = C*x(:,k+1); % Linear measurement equation
end
%% Results
figure; plot(t, x(1,:)); xlabel('Time (sec)'); ylabel('Position'); % Plots position with time
figure; plot(t, x(2,:)); xlabel('Time (sec)'); ylabel('Velocity'); % Plots velocity with time
figure; plot(t, x(3,:)); xlabel('Time (sec)'); ylabel('Acceleration'); % Plots acceleration with time
figure; plot(t, u); xlabel('Time (sec)'); ylabel('Input'); % Plots input with time