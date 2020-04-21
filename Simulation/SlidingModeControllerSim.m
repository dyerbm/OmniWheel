%% Setting up workspace
close all; % Close all figures and windows
clear; % Clear workspace
clc; % Clears screen
%% Initializing parameters
tf = 20; % Final time in simulation
T = 2e-5; % Sample rate
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
   0 0 1]*1e-1;

lambda = [7 0 0;
          0 7 0;
          0 0 6]*1e0;
% lambda=5

%% Motor parameters
n_m = 2; % Number of states
m_m = 1; % Number of measurements
p_m = 1; % Number of inputs
num_m=4; %Number of Motors
J = 0.001;
b = 1;
K_m = 1;
R = 0.6;
L = 0.01;
A_c=[-b/J K_m/J;-K_m/L -R/L];
B_c=[0; 1/L];
C_c=[1 0];
A_d=A_c*T+eye(n_m); %discrete system matrix
B_d=B_c*T; %Discrete input matrix
C_d=C_c; %Discrete Measurement Matrix
A_m=blkdiag(A_d,A_d,A_d,A_d);
B_m=blkdiag(B_d,B_d,B_d,B_d);
C_m=blkdiag(C_d,C_d,C_d,C_d); %Measurement Matrix

x_m = zeros(n_m*num_m, length(t)); % Initializes states to zero
z_m = zeros(m_m*num_m, length(t)); % Initializes measurements to zero
u_m = randn(p_m*num_m, length(t)); % Random input
e_m= z_m;
e_d_m=0;
eint_m=0;

K_p=-20;
K_d=-2;
K_i=-1;

%% Set initial conditions

for k=1:length(t) %Fill in desired path
     x_d(:,k)=[0;0;0]; %stabilize to the origin
%     x_d(:,k)=[k*T*0.5;k*T*0.5;k*T*0.5]; %follow linear trajectory
     
%      if rem(((k)*T),3)==0 %random locations generator
%          x_d(:,k)=-0.5+1*rand(3,1);
%      elseif k~=1
%          x_d(:,k)=x_d(:,k-1);
%      end
         
%      rose = 4; %parameter to create number of rose pedals
%      x_d(:,k)=0.5*[cos(rose*k*T*2*pi/30)*cos(k*T*2*pi/30); cos(rose*k*T*2*pi/30)*sin(k*T*2*pi/30);sin(k*T*2*pi)]; %create rose path
%      if k~=1 
%          xdot_d(:,k)=(x_d(:,k)-x_d(:,k-1))/T; 
%      end

%      x_d(:,k)=0.5*[sin(k*T*2*pi/12)+cos(k*T*2*pi/14);sin(k*T*2*pi/5)+cos(k*T*2*pi/4);0]; %follow linear trajectory

%      vs(:,k)=-0.5+1*rand(4,1); %set the amount of real slip
end

x(:,1)=[1;1;pi]; %any initial condition
%x(:,1)=x_d(:,1); %start at the proper position

z(:,1)=x(:,1); %set first measurement


%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    e(:,k) = z(:,k)-x_d(:,k); %calculate the error
    eint = sum(e,2)*T; %calculate the integral error term
    if k>2/T %calculate integral error term over previous 2 seconds
        eint=sum(e(:,k-2/T:k),2);
    end
    s=e(:,k)+lambda*eint; %calculate 
    u(:,k+1) = 1/wr*(B(x(:,k))*(-lambda*e(:,k)+xdot_d(:,k))-vshat(:,k))-B(x(:,k))*K*sign(s);
    %limit maximum velocity
    if max(abs(u(:,k+1)))>40
       u(:,k+1)=u(:,k+1)/max(abs(u(:,k+1)))*40;
    end

    u(:,k+1)=[40;40;40;40];%force the controller
    
    e_m(:,k)=z_m(:,k)-u(:,k)/40;
    eint_m=sum(e_m,2)*T;
    if k>2/T %calculate integral error term over previous 2 seconds
        eint_m=sum(e_m(:,k-2/T:k),2);
    end
    if k~=1
       e_d_m = (e_m(:,k)-e_m(:,k-1))/T;
    end
    u_m(:,k+1)=K_p*e_m(:,k)+K_d*e_d_m+K_i*eint_m;%calculate voltages for each motor
    if max(abs(u_m(:,k+1)))>12
       u_m(:,k+1)=u_m(:,k+1)/max(abs(u_m(:,k+1)))*12;
    end
    
    %u_m(:,k+1)=u(:,k+1)/40*12;%force linear controller
    
    u_m(:,k+1)
    
    x_m(:,k+1)=A_m*x_m(:,k)+B_m*u_m(:,k+1);
    z_m(:,k+1)=C_m*x_m(:,k+1);
    x_m(:,k+1);
    
    
    x(:,k+1) = x(:,k) + Binv(x(:,k))*(wr*u_m(:,k+1)+vs(:,k))*T; % Linear state space equation
    z(:,k+1) = C*x(:,k+1); % Linear measurement equation
end
%% Results
for k = 1:n %calculate RSME
    RMSE(k) = (sum(e(k,:).^2)/length(t))^0.5; % Calculates the RMSE for EKF using square error values calculated in the for loop
end
RMSE=RMSE


figure; plot(t, x(1,:));hold on;plot(t, x_d(1,:)); xlabel('Time (sec)');hold off;legend('real','desired') % Plots position with time
figure; plot(t, x(2,:));hold on;plot(t, x_d(2,:)); xlabel('Time (sec)');hold off;legend('real','desired') % Plots velocity with time
figure; plot(t, x(3,:));hold on;plot(t, x_d(3,:)); xlabel('Time (sec)');hold off;legend('real','desired') % Plots acceleration with time
figure; plot(t, u); xlabel('Time (sec)'); ylabel('Input'); % Plots input with time
figure; plot(x(1,:), x(2,:)); hold on; plot(x_d(1,:), x_d(2,:)); xlabel('x position (m)');ylabel('y poisition(m)'); hold off; legend('real','desired') % x-y plot of position vs real