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
xNon = @(y) [y(1) + T*y(2); % State equation 1
    y(2) + T*y(3); % State equation 2
    (1 - T*((a2*V0+M*Beta*L)/(M*V0)))*y(3) - T*((A^2*Beta+a2*L*Beta)/(M*V0))*y(2) - T/(M*V0)*(2*a1*V0*y(2)*y(3)+Beta*L*(a1*y(2)^2+a3))*sign(y(2)) + T*((A*Beta)/(M*V0))*y(5); % State equation 3
    (1/A)*(a2*y(2) + (a1*y(2)^2+a3)*sign(y(2)))]; % State equation 4
%% Simulate nonlinear dynamics
for k = 1:length(t)-1 % Simulating nonlinear dynamics
    if k == 4.5/T % Changing system parameters half-way
        L = 6.01e-11; % Parameter (major leakage - table 2)
        QL = 1.47e-5; % Parameter (major flow rate - table 2)
        a1 = 4.462e6; % Parameter (major friction 1 - table 3)
        a2 = 1.863e4; % Parameter (major friction 2 - table 3)
        a3 = 551; % Parameter (major friction 3 - table 3)
    end
    u(k) = Dp*wp(k) - sign(x(4,k))*QL; % Calculates control input (equation 4.4)
    %     x(1,k+1) = x(1,k) + T*x(2,k); % State equation 1 (equation 4.1)
    %     x(2,k+1) = x(2,k) + T*x(3,k); % State equation 2 (equation 4.2)
    %     x(3,k+1) = (1 - T*((a2*V0+M*Beta*L)/(M*V0)))*x(3,k) - T*((A^2*Beta+a2*L*Beta)/(M*V0))*x(2,k) - T/(M*V0)*(2*a1*V0*x(2,k)*x(3,k)+Beta*L*(a1*x(2,k)^2+a3))*sign(x(2,k)) + T*((A*Beta)/(M*V0))*u(k); % State equation 3 (equation 4.3)
    %     x(4,k+1) = (1/A)*(a2*x(2,k) + (a1*x(2,k)^2+a3)*sign(x(2,k))); % State equation 4 (equation 4.7)
    x(:,k+1) = xNon([x(:,k); u(k)]); % Nonlinear system equation
    z(:,k+1) = C*x(:,k+1); % Measurement equation
end
%% Results/plots
figure; plot(t,u,'LineWidth',2); xlabel('Time (sec)'); ylabel('Input signal'); % Input with time
figure; plot(t,x(1,:),'LineWidth',2); xlabel('Time (sec)'); ylabel('Position (m)'); % Position with time
figure; plot(t,x(2,:),'LineWidth',2); xlabel('Time (sec)'); ylabel('Velocity (m/s)'); % Velocity with time
figure; plot(t,x(3,:),'LineWidth',2); xlabel('Time (sec)'); ylabel('Acceleration (m/s^2)'); % Acceleration with time
figure; plot(t,x(4,:),'LineWidth',2); xlabel('Time (sec)'); ylabel('Diff. Pressure (Pa)'); % Diff. pressure with time