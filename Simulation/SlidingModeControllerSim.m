%% Setting up workspace
close all; % Close all figures and windows
clear; % Clear workspace
clc; % Clears screen
%% Initializing parameters
tf = 30; % Final time in simulation
T = 2e-2; % Sample rate
t = 0:T:tf; % Time vector
n = 3; % Number of states
m = 3; % Number of measurements
p = 4; % Number of Inputs
x = zeros(n, length(t)); % Initializes states to zero
x_d = x; %Initializes desired x states
xdot_d = x; %initialises desired xdot states
z = zeros(m, length(t)); % Initializes measurements to zero
e = x; %set up error term
u = zeros(p, length(t)); % Initializes controller
vs = zeros(p,length(t)); %initialize slip values
vshat = vs; %intialized measured slip values

rr=0.195; %define robot radius
wr = 0.03175; %define wheel radius

%A = [-cos(z(3:1)]; % System matrix
% B = @(y)(sqrt(2)/2)*[cos(y(3))+sin(y(3)) -cos(y(3))+sin(y(3)) rr;
%     cos(y(3))-sin(y(3)) cos(y(3))+sin(y(3)) rr;
%     -cos(y(3))-sin(y(3)) cos(y(3))-sin(y(3)) rr;
%     -cos(y(3))+sin(y(3)) -cos(y(3))-sin(y(3)) rr]; % Input matrix
% Binv = @(y)(sqrt(2)/4)*[cos(y(3))-sin(y(3)) cos(y(3))+sin(y(3)) -cos(y(3))+sin(y(3)) -cos(y(3))-sin(y(3));
%     -cos(y(3))-sin(y(3)) cos(y(3))-sin(y(3)) cos(y(3))+sin(y(3)) -cos(y(3))+sin(y(3));
%     1/(sqrt(2)*rr) 1/(sqrt(2)*rr) 1/(sqrt(2)*rr) 1/(sqrt(2)*rr)];
Binv = @(y)[(-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))-sin(y(3)))/(2*sqrt(2));
    (cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2));
    1/(4*rr), 1/(4*rr), 1/(4*rr), 1/(4*rr)]; % Input matrix
B = @(y)[-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    -((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr];

C = eye(m); % Measurement matrix


K= [1 0 0;
   0 1 0;
   0 0 1]*1e0;

lambda = [7 0 0;
          0 7 0;
          0 0 6]*1e1;
% lambda=5

%% Set initial conditions

for k=1:length(t) %Fill in desired path
%     x_d(:,k)=[0;0;0];
     %x_d(:,k)=[k*T*0.5;k*T*0.5;k*T*0.5];
     
     rose = 4; %parameter to create number of rose pedals
     x_d(:,k)=0.5*[cos(rose*k*T*2*pi/30)*cos(k*T*2*pi/30); cos(rose*k*T*2*pi/30)*sin(k*T*2*pi/30);sin(k*T*2*pi)]; %create rose path
     if k~=1 
         xdot_d(:,k)=(x_d(:,k)-x_d(:,k-1))/T; 
     end
end

x(:,1)=[0;0;pi]; %any initial condition
x(:,1)=x_d(:,1); %start at the proper position

z(:,1)=x(:,1); %set first measurement


%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    %u(:,k+1) = [sin(pi*k*T);cos(pi*k*T);-sin(pi*k*T);-cos(pi*k*T)];% calculate controller
    e(:,k) = z(:,k)-x_d(:,k); %calculate the error
    e(:,k)
%     if k>2
%         edot = (e(:,k-1)-e(:,k))/T; %calculate the derivative error term
%     else
%         edot = [0;0;0];
%     end
    eint = sum(e,2)*T; %calculate the integral error term
    if k>2/T %calculate integral error term over previous 2 seconds
        eint=sum(e(:,k-2/T:k),2);
    end
    
    s=e(:,k)+lambda*eint; %calculate 
    
    %u(:,k+1) = 1/wr*(B(x(:,k))*edot-vs(:,k))-B(x(:,k))*K*sign(e);
    u(:,k+1) = 1/wr*(B(x(:,k))*(-lambda*e(:,k)+xdot_d(:,k))-vshat(:,k))-B(x(:,k))*K*sign(s);
    
    %limit maximum velocity
    if max(abs(u(:,k+1)))>40
       u(:,k+1)=u(:,k+1)/max(abs(u(:,k+1)))*40;
       u(:,k+1)
    end
    
    
    x(:,k+1) = x(:,k) + Binv(x(:,k))*(wr*u(:,k+1)+vs(:,k))*T; % Linear state space equation
    z(:,k+1) = C*x(:,k+1); % Linear measurement equation
end
%% Results
for k = 1:n %calculate RSME
    RMSE(k) = (sum(e(k,:).^2)/length(t))^0.5; % Calculates the RMSE for EKF using square error values calculated in the for loop
end


figure; plot(t, x(1,:));hold on;plot(t, x_d(1,:)); xlabel('Time (sec)');hold off;legend('real','desired') % Plots position with time
figure; plot(t, x(2,:));hold on;plot(t, x_d(2,:)); xlabel('Time (sec)');hold off;legend('real','desired') % Plots velocity with time
figure; plot(t, x(3,:));hold on;plot(t, x_d(3,:)); xlabel('Time (sec)');hold off;legend('real','desired') % Plots acceleration with time
figure; plot(t, u); xlabel('Time (sec)'); ylabel('Input'); % Plots input with time
figure; plot(x(1,:), x(2,:)); hold on; plot(x_d(1,:), x_d(2,:)); xlabel('x position (m)');ylabel('y poisition(m)'); hold off; legend('real','desired') % x-y plot of position vs real