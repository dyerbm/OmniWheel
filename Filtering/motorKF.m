%% Setting up workspace
close all; % Close all figures and windows
clear; % Clear workspace
clc; % Clears screen
%% Initializing parameters
tf = 10; % Final time in simulation
T = 2e-2; % Sample rate
t = 0:T:tf; % Time vector
n = 2; % Number of states
m = 1; % Number of measurements
p = 1; % Number of inputs
A_c = [-0.1/0.001 , 0.01/0.001; -0.01/0.5 , -1/0.5]; % System matrix for single motor in continuous time
B_c = [0;1/0.5]; % Input matrix for single motor in continuous time
C_c = [1,0]; % Measurement matrix for single motor in continuous time


A= exp(A_c*T); %Discrete system matrix for single motor
B=inv(A_c)*(A-eye(2))*B_c;
A=A_c*T+eye(n)
B=B_c*T
%B=integral(@(x) exp(A_c*x),0,T,'ArrayValued',true)*B_c; %Discrete Input Matrix for single motor
C=C_c; %Discrete Measurement Matrix

Q = diag([0.009 0.08])*T; % Defines Q
R = diag([0.1])*T; % Defines R
% Q= integral(@(x) exp(A_c*x)*Q_c*exp(transpose(A_c)*x),0,T,'ArrayValued',true);
% R = R_c/T;
w = mvnrnd(zeros(length(t),n),Q)'*T; % Defines w
v = mvnrnd(zeros(length(t),m),R)'*T; % Defines v
x = zeros(n, length(t)); % Initializes states to zero
z = zeros(m, length(t)); % Initializes measurements to zero
u = randn(1, length(t)); % Random input
u = u./max(abs(u)); % Normalizes input between -1 and +1
u=12*sin(t*2*pi);%sinusoidal input
x_kf = x;  % Initializes state estimates
P_kf = Q; % Initializes state error covariance

RMSE = zeros(n,1); % Initializes RMSE vector
error = (x(:,1) - x_kf(:,1)).^2;

%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    x(:,k+1) = A*x(:,k) + B*u(k) + w(:,k); % Linear state space equation
    z(:,k+1) = C*x(:,k+1) + v(:,k+1); % Linear measurement equation
    [x_kf(:,k+1), P_kf(:,:,k+1)] = kf(x_kf(:,k), z(:,k+1), u(k), P_kf(:,:,k), A, B, C, Q, R); % Calls KF function to estimate states  

    error(:,k+1) = (x(:,k+1) - x_kf(:,k+1)).^2;
end

for k = 1:n
    RMSE(k) = (sum(error(k,:))/length(t))^0.5; % Calculates the RMSE for EKF using square error values calculated in the for loop
end

fprintf('RMSE (KF): %s\n', RMSE)
%% Results
figure; plot(t, x(1,:)); hold all; plot(t, x_kf(1,:)); xlabel('Time (sec)'); ylabel('Position'); legend('True','KF');hold off; % Plots position and estimated position with time
figure; plot(t, x(2,:)); hold all; plot(t, x_kf(2,:));xlabel('Time (sec)'); ylabel('Velocity'); legend('True','KF');hold off; % Plots velocity with time
% figure; plot(t, u); xlabel('Time (sec)'); ylabel('Input'); % Plots input with time