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
Ahat = A; % System matrix for filters
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
x_kf = x;  % Initializes KF estimates
x_svsf = x_kf; % Initializes SVSF estimates
P_kf = 10*Q; % Initializes KF state error covariance
P_svsf = P_kf; % Initializes SVSF state error covariance
err_svsf = z - C*x_svsf; % Initializes SVSF measurement error
RMSE = zeros(n,2); % Initializes RMSE vector for KF and SVSF
Squared_Error =[(x(:,1)-x_kf(:,1)).^2 (x(:,1)-x_svsf(:,1)).^2]; % Initializes squared error
%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    if k == 0.5/T
        A = [1 T 0; 0 1 T; -240 -28 0.9418]; % Faulty system
    end
    x(:,k+1) = A*x(:,k) + B*u(k) + w(:,k); % Linear state space equation
    z(:,k+1) = C*x(:,k+1) + v(:,k+1); % Linear measurement equation
    [x_kf(:,k+1), P_kf(:,:,k+1)] = kf(x_kf(:,k), z(:,k+1), u(k), P_kf(:,:,k), Ahat, B, C, Q, R); % Calls KF function to estimate states  
    [x_svsf(:,k+1), P_svsf(:,:,k+1), err_svsf(:,k+1)] = svsf(x_svsf(:,k), z(:,k+1), err_svsf(:,k), u(k), P_svsf(:,:,k), Ahat, B, C, Q, R); % Calls SVSF function to estimate states  
    Squared_Error(:,:,k+1) =[(x(:,k+1)-x_kf(:,k+1)).^2 (x(:,k+1)-x_svsf(:,k+1)).^2]; % Calculates squared error
end
%% Results
for k = 1:n
    RMSE(k,1) = (sum(Squared_Error(k,1,:))/length(t))^0.5; % Calculates RMSE for KF
    RMSE(k,2) = (sum(Squared_Error(k,2,:))/length(t))^0.5; % Calculates RMSE for SVSF
end
State = ['1'; '2'; '3']; % Creates labels for results table
Results_RMSE = table(State, RMSE(:,1), RMSE(:,2)); % Prepares RMSE results into a table
Results_RMSE.Properties.VariableNames = {'State', 'KF', 'SVSF'}; % Label the columns
fprintf('Simulation Results: \n\n'); % Displays title for results
disp(Results_RMSE); % Displays RMSE results table
figure; plot(t, x(1,:)); hold all; plot(t, x_kf(1,:)); plot(t, x_svsf(1,:)); xlabel('Time (sec)'); ylabel('Position'); legend('True','KF','SVSF'); % Plots position and estimated position with time
% figure; plot(t, x(2,:)); xlabel('Time (sec)'); ylabel('Velocity'); % Plots velocity with time
% figure; plot(t, x(3,:)); xlabel('Time (sec)'); ylabel('Acceleration'); % Plots acceleration with time
% figure; plot(t, u); xlabel('Time (sec)'); ylabel('Input'); % Plots input with time