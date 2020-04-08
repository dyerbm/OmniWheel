%% Setting up workspace
clear;
close all;
clc;
%% Initialize parameters and definitions
Tf = 9; % Final simulation time
T = 1e-3; % Sample rate
t = 0:T:Tf; % Time vector
n = 4; % Number of states
m = 4; % Number of measurements
A = 1.52e-3; % Parameter (area - table 1)
Dp = 5.57e-7; % Parameter (piston diameter - table 1)
M = 7.736; % Parameter (actuator mass - table 1)
V0 = 1.08e-3; % Parameter (hydraulic volume - table 1)
Beta = 2.07e8; % Parameter (effective bulk modulus - table 1)
L = 4.78e-12; % Parameter (normal leakage - table 2)
QL = 2.41e-6; % Parameter (normal flow rate - table 2)
a1 = 6.589e4; % Parameter (normal friction 1 - table 3)
a2 = 2.144e3; % Parameter (normal friction 2 - table 3)
a3 = 436; % Parameter (normal friction 3 - table 3)
C = eye(m); % Defines measurement matrix
x = zeros(n,length(t)); % Initialize states
z = zeros(m,length(t)); % Initialize measurements
u = zeros(1,length(t)); % Initialize input
wp = -100*square(pi*t); % Defines angular velocity of pump
Q = 1e-9*eye(n); % Defines system noise covariance
R = 1e3*Q; % Defines measurement noise covariance
P_ekf = 10*Q; % Iniitialize EKF state error covariance
x_ekf = x; % Initialize EKF estimates
w = mvnrnd(zeros(n,1), Q, length(t))'; % Defines system noise (zero mean and covariance Q)
v = mvnrnd(zeros(m,1), R, length(t))'; % Defines measurement noise (zero mean and covariance R)
xNon = @(y) [y(1) + T*y(2); % State equation 1
    y(2) + T*y(3); % State equation 2
    (1 - T*((a2*V0+M*Beta*L)/(M*V0)))*y(3) - T*((A^2*Beta+a2*L*Beta)/(M*V0))*y(2) - T/(M*V0)*(2*a1*V0*y(2)*y(3)+Beta*L*(a1*y(2)^2+a3))*sign(y(2)) + T*((A*Beta)/(M*V0))*y(5); % State equation 3
    (1/A)*(a2*y(2) + (a1*y(2)^2+a3)*sign(y(2)))]; % State equation 4
Alin = @(y) [1 T 0 0; % Linearized system matrix A based on the calculated Jacobian (note that we made it a function since the states change with time and need to be calculated at each time step)
    0 1 T 0;
    0 -(T*(Beta*A^2 + Beta*L*a2 + 2*V0*a1*y(3)*sign(y(2)) + 2*Beta*L*a1*y(2)*sign(y(2))))/(M*V0) 1 - (2*T*a1*y(2)*sign(y(2)))/M - (T*(V0*a2 + Beta*L*M))/(M*V0) 0;
    0 (a2 + 2*a1*y(2)*sign(y(2)))/A 0 0];
RMSE = zeros(n,1); % Initializes RMSE vector
error = (x(:,1) - x_ekf(:,1)).^2; % Initializes squared error
%% Simulate nonlinear dynamics
for k = 1:length(t)-1 % Simulating nonlinear dynamics
    u(k) = Dp*wp(k) - sign(x(4,k))*QL; % Calculates control input (equation 4.4)
    x(:,k+1) = xNon([x(:,k); u(k)]) + w(:,k); % Nonlinear system equation
    z(:,k+1) = C*x(:,k+1) + v(:,k+1); % Measurement equation
    [x_ekf(:,k+1), P_ekf(:,:,k+1)] = ekf(x_ekf(:,k), z(:,k+1), u(k), P_ekf(:,:,k), xNon, Alin, C, Q, R); % Calls EKF function to estimate states  
    error(:,k+1) = (x(:,k+1) - x_ekf(:,k+1)).^2; % Calculates squared error
end
%% Results/plots
for k = 1:n
    RMSE(k) = (sum(error(k,:))/length(t))^0.5; % Calculates the RMSE for EKF using square error values calculated in the for loop
end
fprintf('RMSE (EKF): %s\n', RMSE); % Displays RMSE results
% figure; plot(t,u,'LineWidth',2); xlabel('Time (sec)'); ylabel('Input signal'); % Input with time
figure; plot(t,x(1,:),'LineWidth',2); hold all; plot(t,x_ekf(1,:),'--','LineWidth',2); xlabel('Time (sec)'); ylabel('Position (m)'); % Position with time
% figure; plot(t,x(2,:),'LineWidth',2); xlabel('Time (sec)'); ylabel('Velocity (m/s)'); % Velocity with time
% figure; plot(t,x(3,:),'LineWidth',2); xlabel('Time (sec)'); ylabel('Acceleration (m/s^2)'); % Acceleration with time
% figure; plot(t,x(4,:),'LineWidth',2); xlabel('Time (sec)'); ylabel('Diff. Pressure (Pa)'); % Diff. pressure with time