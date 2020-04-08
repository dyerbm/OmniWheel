%%  Linear system simulation from L02 and L06 used for IMM coding (L10)
clear; % Clears workspace
close all; % Closes all figures
clc; % Clears screen
%% Initialization stage
tf = 1; % Final simulation time (1 sec)
T = 0.001; % Define sample rate
t = 0:T:tf; % Time vector
n = 3; % Number of states
m = 3; % Number of measurements
A1 = [1 T 0; 0 1 T; -557.02 -28.616 0.9418]; % System matrix (normal)
A2 = [1 T 0; 0 1 T; -240 -28 0.9418]; % System matrix (faulty)
A = A1; % Sets system matrix to 'normal' for start of simulation
B = [0; 0; 557.02]; % Input matrix
C = eye(m); % Measurement matrix
Q = diag([1e-5 1e-3 1e-1]); % Defines Q
R = diag([1e-4 1e-2 1]); % Defines R
w = mvnrnd(zeros(length(t),n),Q)'; % Defines wc
v = mvnrnd(zeros(length(t),m),R)'; % Defines v
x = zeros(n, length(t)); % Initialize x to zero
z = zeros(m, length(t)); % Initialize z to zero
u = randn(1, length(t)); % Random input
u = u./max(abs(u)); % Normalized input between -1 and +1
u(0.5/T:end) = u(0.5/T:end) + 1; % Superimposes +1 on the second-half of the input
x_imm = x; % Initializes state estimates (KF)
P_imm = 10*Q; % Initializes state error covariance
RMSE = zeros(n,1); % Initializes RMSE vector for IMM
Squared_Error = (x(:,1) - x_imm(:,1)).^2; % Initializes squared error (difference between true and estimated states)
mu = [0.90; 0.10]; % Initializes mu vector (probabilities for A1 and A2)
%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates the system for 1 second
    if k == 0.5/T
        A = A2; % Introduces modeling uncertainty half-way through the simulation
    end
    x(:,k+1) = A*x(:,k) + B*u(k) + w(:,k); % Linear state space equation
    z(:,k+1) = C*x(:,k+1) + v(:,k+1); % Measurement equation
    [x_imm(:,k+1), P_imm(:,:,k+1), mu(:,k+1)] = imm(z(:,k+1), u(k), A1, A2, B, C, Q, R, k); % Calls the KF function to estimate states
    Squared_Error(:,:,k+1) = (x(:,k+1) - x_imm(:,k+1)).^2; % Calculates squared error at each time step (for IMM)
end
%% Results
for k = 1:n % For loop to calculate the EKF RMSE
    RMSE(k,1) = (sum(Squared_Error(k,1,:))/length(t))^0.5; % Calculates the RMSE for KF using squared error values calculated in the for loop (KF)
end
State = ['1'; '2'; '3']; % Creates label for table of RMSE results
Results_RMSE = table(State,RMSE(:,1)); % Presents the RMSE results in the form of a table
Results_RMSE.Properties.VariableNames = {'State','KF'}; % Labels the columns
fprintf('Simulation Results: \n\n' ); % Displays title for results
disp(Results_RMSE); % Displays RMSE results table
figure; plot(t, x(1,:), 'LineWidth', 2); hold all; plot(t, x_imm(1,:), '--', 'LineWidth', 1.2); xlabel('Time (sec)'); ylabel('Position (m)'); legend('True','IMM-KF Estimate','Location','NorthWest');  % State 1 plot
figure; plot(t,mu(1,:), 'LineWidth', 2); hold all; plot(t, mu(2,:),'LineWidth',2); xlabel('Time (sec)'); ylabel('Mode Probabilities'); legend('Mu_1','Mu_2'); % IMM mode probabilities