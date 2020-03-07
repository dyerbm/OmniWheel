%% Setting up workspace
close all; % Close all figures and windows
clear; % Clear workspace
clc; % Clears screen
%% Initializing parameters
tf = 10; % Final time in simulation
T = 2e-2; % Sample rate
t = 0:T:tf; % Time vector
n = 3; % Number of states
m = 3; % Number of measurements
p = 4; % Number of controlable states
x = zeros(n, length(t)); % Initializes states to zero
x_d = x; %Initializes desired x states
z = zeros(m, length(t)); % Initializes measurements to zero
u = zeros(p, length(t)); % Initializes controller
v = u; %initialize linearized controller

rr=0.195; %define robot radius

%A = [-cos(z(3:1)]; % System matrix
B = @(y)[(-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))-sin(y(3)))/(2*sqrt(2));
    (cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2));
    1/(4*rr), 1/(4*rr), 1/(4*rr), 1/(4*rr)]; % Input matrix
Binv = @(y)[-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    -((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr];

C = eye(m); % Measurement matrix

Kp=[0,0,0,0; %Proportional Gain
    0,0,0,0;
    0,0,0,0]
Kd=[0,0,0,0; %Derivative gain
    0,0,0,0;
    0,0,0,0]
Ki=[0,0,0,0; %Integral gain
    0,0,0,0;
    0,0,0,0]

%% Set initial conditions
x(:,1)=[1;0;0];

for k=1:length(t) %Fill in desired path
    xd(:,k)=0;
end



%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    u(:,k+1) = [sin(pi*k*T);cos(pi*k*T);-sin(pi*k*T);-cos(pi*k*T)];% calculate controller
    %v(:,k+1) = Kp*z(:,k+1);
    %u(:,k+1)=Binv*v(:,k+1);
    
    x(:,k+1) = x(:,k) + B(x(:,k))*u(:,k+1)*T; % Linear state space equation
    z(:,k+1) = C*x(:,k+1); % Linear measurement equation
end
%% Results
figure; plot(t, x(1,:)); xlabel('Time (sec)'); % Plots position with time
figure; plot(t, x(2,:)); xlabel('Time (sec)'); % Plots velocity with time
figure; plot(t, x(3,:)); xlabel('Time (sec)'); % Plots acceleration with time
figure; plot(t, u); xlabel('Time (sec)'); ylabel('Input'); % Plots input with time