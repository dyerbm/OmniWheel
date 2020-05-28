%% Setting up workspace
close all; % Close all figures and windows
%clear; % Clear workspace
%clc; % Clears screen
%% Initializing parameters
tf = 600; % Final time in simulation
T = 2e-2; % Sample rate
t = 0:T:tf; % Time vector

%% Definitions for the Wheels
n_m = 2; % Number of states
m_m = 1; % Number of measurements
p_m = 1; % Number of inputs
num_m=4; %Number of Motors
J = 0.01;
b = 1;
K_m = 1.05;
R = 2.7;
L = 0.0014;
A_c=[-b/J K_m/J;-K_m/L -R/L];
B_c=[0; 1/L];
A_c = [-0.1/0.01 , 0.1/0.01; -0.1/0.01 , -1/0.01]; % System matrix for single motor in continuous time
B_c = [0;1/0.01]; % Input matrix for single motor in continuous time
C_c = [1,0]; % Measurement matrix for single motor in continuous time
A_d=A_c*T+eye(n_m); %discrete system matrix
B_d=B_c*T; %Discrete input matrix
C_d=C_c; %Discrete Measurement Matrix
A_m=blkdiag(A_d,A_d,A_d,A_d);
B_m=blkdiag(B_d,B_d,B_d,B_d);
C_m=blkdiag(C_d,C_d,C_d,C_d); %Measurement Matrix
Q_m = diag([2 0.1 2 0.1 2 0.001 2 0.1])*5e0; % Defines Q
R_m = diag([0.1,0.1,0.1,0.1])*1e-1; % Defines R
w_m = mvnrnd(zeros(length(t),n_m*num_m),Q_m)'*T; % Defines w
v_m = mvnrnd(zeros(length(t),m_m*num_m),R_m)'; % Defines v
x_m = zeros(n_m*num_m, length(t)); % Initializes states to zero
z_m = zeros(m_m*num_m, length(t)); % Initializes measurements to zero
u_m = randn(p_m*num_m, length(t)); % Random input
u_m = u_m./max(abs(u_m)); % Normalizes input between -1 and +1
u_m=[20*sin(t*2*pi);12*sin(t*2*pi);6*sin(t*2*pi);3*sin(t*2*pi)];%sinusoidal input
x_kf_m = x_m;  % Initializes state estimates
P_kf_m = Q_m; % Initializes state error covariance

RMSE_m = zeros(n_m*num_m,1); % Initializes RMSE vector
error_m = (x_m(:,1) - x_kf_m(:,1)).^2;

%% Definitions for the Robot
n_r = 3; % Number of states
m_r = 3; % Number of measurements
p_r = 4; % Number of Inputs
C_r = eye(m_r); % Defines measurement matrix
x_r = zeros(n_r,length(t)); % Initialize states
x_d_r = x_r; %Initializes desired x states
xdot_d_r = x_r; %initialises desired xdot states
e_r = x_r; %set up error term
z_r = zeros(m_r,length(t)); % Initialize measurements
u_r = zeros(p_r,length(t)); % Initialize input
Q_r = 1e-3*eye(n_r); % Defines system noise covariance
R_r = 1e-20*eye(n_r); % Defines measurement noise covariance
P_ekf_r = 10*Q_r; % Iniitialize EKF state error covariance
x_ekf_r = x_r; % Initialize EKF estimates
w_r = mvnrnd(zeros(n_r,1), Q_r, length(t))'*T; % Defines system noise (zero mean and covariance Q)
v_r = mvnrnd(zeros(m_r,1), R_r, length(t))'; % Defines measurement noise (zero mean and covariance R)
rr=0.195; %define robot radius
wr = 0.03175; %define wheel radius
xNon = @(y)[(-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))-sin(y(3)))/(2*sqrt(2));
    (cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2));
    1/(4*rr), 1/(4*rr), 1/(4*rr), 1/(4*rr)]*(wr*[y(4);y(5);y(6);y(7)]+[y(8);y(9);y(10);y(11)])*T+[y(1);y(2);y(3)];
Alin = @(y) [ 1, 0, -T*((2^(1/2)*(y(8) + y(4)*wr)*(cos(y(3)) - sin(y(3))))/4 - (2^(1/2)*(y(10) + y(6)*wr)*(cos(y(3)) - sin(y(3))))/4 - (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(9) + y(5)*wr))/4 + (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(11) + y(7)*wr))/4);
 0, 1, -T*((2^(1/2)*(y(9) + y(5)*wr)*(cos(y(3)) - sin(y(3))))/4 - (2^(1/2)*(y(11) + y(7)*wr)*(cos(y(3)) - sin(y(3))))/4 + (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(8) + y(4)*wr))/4 - (2^(1/2)*(cos(y(3)) + sin(y(3)))*(y(10) + y(6)*wr))/4);
 0, 0, 1];%linearized dynamics input at [x;u;vs]
B_r = @(y)[-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    -((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr];
vs_r = zeros(p_r,length(t)); %initialize slip values
vshat_r = vs_r; %intialized measured slip values
RMSE_r = zeros(n_r,1); % Initializes RMSE vector
RMSE = zeros(n_r,1); % Initializes RMSE vector
error_r = (x_r(:,1) - x_ekf_r(:,1)).^2; % Initializes squared error

u_m=cell2mat(struct2cell(load('u_m.mat')));


%% set up for the controller

for k=1:length(t) %Fill in desired path
       
     rose = 1; %parameter to create number of rose pedals
     %x_d_r(:,k)=200*[cos(rose*k*T*2*pi/30)*cos(k*T*2*pi/30); cos(rose*k*T*2*pi/30)*sin(k*T*2*pi/30);sin(k*T*2*pi)]; %create rose path
     %u_m(:,k)=200*[cos(rose*k*T*2*pi/30)*cos(k*T*2*pi/30); -cos(rose*k*T*2*pi/30)*sin(k*T*2*pi/30);-cos(rose*k*T*2*pi/30)*cos(k*T*2*pi/30); cos(rose*k*T*2*pi/30)*sin(k*T*2*pi/30)];
     
     %u_m(:,k)=200*[cos(rose*k*T*2*pi/30)+2*cos(k*T*2*pi/15); sin(rose*k*T*2*pi/30)+2*sin(k*T*2*pi/15);-cos(rose*k*T*2*pi/30)-cos(k*T*2*pi/15); -sin(rose*k*T*2*pi/30)-sin(k*T*2*pi/15)];
     u_m(:,k)=12*[sin(k*T*2*pi/20);cos(k*T*2*pi/10);-sin(k*T*2*pi/20);-cos(k*T*2*pi/10)]+sin(k*T*2*pi/40)*cos(k*T*2*pi/40);
     
     %u_m(:,k)=100*B_r([0;0;sin(k*T*2*pi/10)])*[cos(rose*k*T*2*pi/30)*cos(k*T*2*pi/30); cos(rose*k*T*2*pi/30)*sin(k*T*2*pi/30);cos(k*T*2*pi)];

%      x_d_r(:,k)=0.5*[sin(k*T*2*pi/12)+cos(k*T*2*pi/14);sin(k*T*2*pi/5)+cos(k*T*2*pi/4);sin(k*T*2*pi/16)+cos(k*T*2*pi/8)]; %follow linear trajectory

     %vs_r(:,k)=-0.1+0.2*rand(4,1); %set the amount of real slip
end

x_r(:,1)=[0;0;0]; %any initial condition
x_r(:,1)=x_d_r(:,1); %start at the proper position

z_r(:,1)=x_r(:,1); %set first measurement
x_ekf_r=x_r(:,1); %correct starting point for ekf

x_m_unfiltered=x_m; %set up other cases
x_nofilter=x_ekf_r;
x_kfonly=x_ekf_r;
x_ekfonly=x_ekf_r;
P_ekfonly=P_ekf_r;
x_SVSF_r=x_ekf_r;
x_SVSFonly=x_ekf_r;
P_SVSF_r=P_ekf_r;
P_SVSFonly=P_ekf_r;
P_SVSFm=P_kf_m;
z_m_real = z_m; %create measurement with no noise for motors

error_nofilter = (x_r(:,1) - x_nofilter(:,1)).^2; % Initializes squared error
error_ekfonly = (x_r(:,1) - x_ekfonly(:,1)).^2; % Initializes squared error
error_kfonly = (x_r(:,1) - x_kfonly(:,1)).^2; % Initializes squared error
error_SVSF = (x_r(:,1) - x_SVSF_r(:,1)).^2; %initialize squared error
error_SVSFonly = (x_r(:,1) - x_SVSFonly(:,1)).^2;

RMSE_nofilter =RMSE_r;
RMSE_ekfonly = RMSE_r;
RMSE_kfonly = RMSE_r;
RMSE_SVSF = RMSE_r;
RMSE_SVSFonly = RMSE_r;

SVSFe=0;
SVSFonlye=0;
SVSFme=0;

%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    %calculate the controller, (wheel voltage)
    
    
    %Filter the motors
    x_m(:,k+1) = A_m*x_m(:,k) + B_m*u_m(:,k) + w_m(:,k); % Linear state space equation
    z_m(:,k+1) = C_m*x_m(:,k+1) + v_m(:,k+1); % Linear measurement equation
    z_m_real(:,k+1) = C_m*x_m(:,k+1);
    
    [x_kf_m(:,k+1), P_kf_m(:,:,k+1)] = kf(x_kf_m(:,k), z_m(:,k+1), u_m(:,k), P_kf_m(:,:,k), A_m, B_m, C_m, Q_m, R_m); % Calls KF function to estimate states
    x_m_unfiltered(:,k+1) = A_m*x_m_unfiltered(:,k)+ B_m*u_m(:,k); %calculate wheel without filter
    %[x_m_unfiltered(:,k+1), P_SVSFm(:,:,k+1), SVSFme] = svsf(x_m_unfiltered(:,k), z_m(:,k+1), SVSFme, u_m(:,k), P_SVSFm(:,:,k), A_m, B_m, C_m, Q_m, R_m); % Calls KF function to estimate states
    
    z_kf_m=C_m*x_kf_m(:,k+1); %get state with kf
    z_m_unfiltered=C_m*x_m_unfiltered(:,k+1); %get state without kf
    
    %% Filter the robot
%     %x_r(:,k+1) = x_r(:,k) + pinv(B_r(x_r(:,k)))*(wr*u_m(:,k+1)+vs_r(:,k))*T;
%     x_r(:,k+1)= xNon([x_r(:,k);z_m(:,k+1);vs_r(:,k)])+w_r(:,k); % Linear state space equation
%     z_r(:,k+1) = C_r*x_r(:,k)+v_r(:,k); % Linear measurement equation
%     
%     
%     [x_ekf_r(:,k+1), P_ekf_r(:,:,k+1)] = ekf(x_ekf_r(:,k), z_r(:,k+1), z_kf_m, vshat_r(:,k), P_ekf_r(:,:,k), xNon, Alin, C_r, Q_r, R_r); % Calls EKF function to estimate states      
%     x_kfonly(:,k+1)=xNon([x_kfonly(:,k);z_kf_m;vs_r(:,k)]);
%     [x_ekfonly(:,k+1), P_ekfonly(:,:,k+1)]=ekf(x_ekfonly(:,k), z_r(:,k+1), z_m_unfiltered, vshat_r(:,k), P_ekfonly(:,:,k), xNon, Alin, C_r, Q_r, R_r);
%     x_nofilter(:,k+1)=xNon([x_nofilter(:,k);z_m_unfiltered;vshat_r(:,k)]);
% %    [x_SVSF_r(:,k+1), P_SVSF_r(:,:,k+1)] = SVSF(x_SVSF_r(:,k),z_r(:,k+1), z_kf_m, vshat_r(:,k), P_SVSF_r(:,:,k), xNon, C_r, Q_r, R_r);

    %x_r(:,k+1) = x_r(:,k) + pinv(B_r(x_r(:,k)))*(wr*u_m(:,k+1)+vs_r(:,k))*T;
    x_r(:,k+1) = xNon([x_r(:,k);z_m_real(:,k+1);vshat_r(:,k)])+w_r(:,k); % Linear state space equation, %%%%%%%%%%%%%check this
    z_r(:,k+1) = C_r*x_r(:,k+1)+v_r(:,k+1); % Linear measurement equation
    
    
    %z_r_ekf=C_r*(xNon([x_ekf_r(:,k);z_kf_m;vs_r(:,k)])+mvnrnd(zeros(n_r,1), 1e-20*eye(3), 1)'*T)+mvnrnd(zeros(n_r,1), 1e-20*eye(3), 1)'; %calculate the position based on the filtered motors
    z_r_ekf=C_r*(xNon([x_ekf_r(:,k);z_kf_m;vs_r(:,k)])+w_r(:,k))+v_r(:,k+1);
    z_r_SVSF=C_r*(xNon([x_SVSF_r(:,k);z_kf_m;vs_r(:,k)])+w_r(:,k))+v_r(:,k+1);
    z_r_onlyekf=C_r*(xNon([x_ekfonly(:,k);z_m_unfiltered;vs_r(:,k)])+w_r(:,k))+v_r(:,k+1); %calculate the position based on the unfiltered motors
    z_r_onlySVSF = C_r*(xNon([x_SVSFonly(:,k);z_m_unfiltered;vs_r(:,k)])+w_r(:,k))+v_r(:,k+1);
    %z_r_ekf=C_r*x_ekf_r(:,k)+v_r(:,k); %calculate the position based on the filtered motors
    %z_r_onlyekf=C_r*x_ekfonly(:,k)+v_r(:,k); %calculate the position based on the unfiltered motors
%     if k>0
%         z_r_ekf=z_r(:,k+1)
%         z_r_onlyekf=z_r(:,k+1)
%     end 
    
    %[x_ekf_r(:,k+1), P_ekf_r(:,:,k+1)] = ekf(x_ekf_r(:,k), z_r_ekf, z_kf_m, vshat_r(:,k), P_ekf_r(:,:,k), xNon, Alin, C_r, 1e-20*eye(3), 1e-20*eye(3)); % Calls EKF function to estimate states      
    [x_ekf_r(:,k+1), P_ekf_r(:,:,k+1)] = ekf(x_ekf_r(:,k), z_r_ekf, z_kf_m, vshat_r(:,k), P_ekf_r(:,:,k), xNon, Alin, C_r, Q_r, R_r);
    x_kfonly(:,k+1)=xNon([x_kfonly(:,k);z_kf_m;vs_r(:,k)]);
    [x_ekfonly(:,k+1), P_ekfonly(:,:,k+1)]=ekf(x_ekfonly(:,k), z_r_onlyekf, z_m_unfiltered, vshat_r(:,k), P_ekfonly(:,:,k), xNon, Alin, C_r, Q_r, R_r);
    x_nofilter(:,k+1)=xNon([x_nofilter(:,k);z_m_unfiltered;vshat_r(:,k)]);
    %[x_SVSF_r(:,k+1), P_SVSF_r(:,:,k+1)] = SVSF(x_SVSF_r(:,k),z_r_SVSF, z_kf_m, vshat_r(:,k), P_SVSF_r(:,:,k), xNon, C_r, Q_r, R_r);
    %[x_SVSFonly(:,k+1), P_SVSFonly(:,:,k+1)] = SVSF(x_SVSFonly(:,k),z_r_onlySVSF, z_m_unfiltered, vshat_r(:,k), P_SVSFonly(:,:,k), xNon, C_r, Q_r, R_r);
    [x_SVSF_r(:,k+1), P_SVSF_r(:,:,k+1),SVSFe] = svsfn(x_SVSF_r(:,k),z_r_SVSF, SVSFe, z_kf_m, vshat_r(:,k), P_SVSF_r(:,:,k), xNon, Alin, C_r, Q_r, R_r);
    [x_SVSFonly(:,k+1), P_SVSFonly(:,:,k+1),SVSFonlye] = svsfn(x_SVSFonly(:,k),z_r_onlySVSF,SVSFonlye, z_m_unfiltered, vshat_r(:,k), P_SVSFonly(:,:,k), xNon, Alin, C_r, Q_r, R_r);
    
    
    
%     z_r_ekf=C_r*x_ekf_r(:,k+1)+v_r(:,k+1) %calculate the position based on the filtered motors
%     z_r_onlyekf=C_r*x_ekfonly(:,k+1)+v_r(:,k+1) %calculate the position based on the unfiltered motors
    
    error_SVSF(:,k+1) = (x_r(:,k+1)-x_SVSF_r(:,k+1)).^2;
    error_SVSFonly(:,k+1) = (x_r(:,k+1)-x_SVSFonly(:,k+1)).^2;
    error_r(:,k+1) = (x_r(:,k+1) - x_ekf_r(:,k+1)).^2;
    error_m(:,k+1) = (x_m(:,k+1) - x_kf_m(:,k+1)).^2;
    error_nofilter(:,k+1) = (x_r(:,k+1) - x_nofilter(:,k+1)).^2; % Initializes squared error
    error_ekfonly(:,k+1) = (x_r(:,k+1) - x_ekfonly(:,k+1)).^2; % Initializes squared error
    error_kfonly(:,k+1) = (x_r(:,k+1) - x_kfonly(:,k+1)).^2; % Initializes squared error
end

for k = 1:n_m*num_m
    RMSE_m(k) = (sum(error_m(k,:))/length(t))^0.5; % Calculates the RMSE for EKF using square error values calculated in the for loop
end
for k = 1:n_r
    RMSE_r(k) = (sum(error_r(k,:))/length(t))^0.5; % Calculates the RMSE for EKF using square error values calculated in the for loop
    RMSE_nofilter(k) = (sum(error_nofilter(k,:))/length(t))^0.5;
    RMSE_ekfonly(k) = (sum(error_ekfonly(k,:))/length(t))^0.5;
    RMSE_kfonly(k) = (sum(error_kfonly(k,:))/length(t))^0.5;
    RMSE_SVSF(k) = (sum(error_SVSF(k,:))/length(t))^0.5;
    RMSE_SVSFonly(k) = (sum(error_SVSFonly(k,:))/length(t))^0.5;
end
fprintf('RMSE (KF): %s\n', RMSE_m)
fprintf('RMSE (EKF): %s\n', RMSE_r)
fprintf('RMSE (nofilter): %s\n', RMSE_nofilter)
fprintf('RMSE (EKFonly): %s\n', RMSE_ekfonly)
fprintf('RMSE (KFonly): %s\n', RMSE_kfonly)
fprintf('RMSE (SVSF): %s\n', RMSE_SVSF)
fprintf('RMSE (SVSFonly): %s\n', RMSE_SVSFonly)
%% Results
figure; plot(t, x_m(1,:)); hold all; plot(t, x_m_unfiltered(1,:)); plot(t, x_kf_m(1,:)); xlabel('Time (sec)'); ylabel('Angular Velocity (rad/s)'); legend('True','No Filter','KF');hold off; % Plots position and estimated position with time
figure; plot(t, x_m(2,:)); hold all; plot(t, x_m_unfiltered(2,:)); plot(t, x_kf_m(2,:));xlabel('Time (sec)'); ylabel('Current (A)'); legend('True','KF','No Filter');hold off; % Plots velocity with time
% figure; plot(t, u); xlabel('Time (sec)'); ylabel('Input'); % Plots input with time

%figure; plot(t, x_r(1,:)); hold all; plot(t, x_ekf_r(1,:));plot(t, x_d_r(1,:)); xlabel('Time (sec)'); ylabel('x position (m)'); legend('True','KF','desired');hold off; % Plots position and estimated position with time
% figure; plot(x_ekf_r(1,:), x_ekf_r(2,:)); hold all; plot(x_r(1,:), x_r(2,:));plot(x_d_r(1,:), x_d_r(2,:)); xlabel('x (m)'); ylabel('y (m)');xlim([-1.1,1.1]);ylim([-1.1,1.1]); legend('Filtered','True','Desired');hold off; % Plots position and estimated position with time
% figure; plot(t, x_ekf_r(3,:)); hold all; plot(t, x_r(3,:));plot(t, x_d_r(3,:)); xlabel('time (s)'); ylabel('theta (rad)'); legend('Filtered','True','Desired');hold off;


figure; plot(x_r(1,:), x_r(2,:));hold all;plot(x_ekf_r(1,:), x_ekf_r(2,:));plot(x_ekfonly(1,:), x_ekfonly(2,:));plot(x_kfonly(1,:), x_kfonly(2,:));plot(x_nofilter(1,:), x_nofilter(2,:));plot(x_SVSF_r(1,:), x_SVSF_r(2,:));plot(x_SVSFonly(1,:), x_SVSFonly(2,:)); xlabel('x (m)'); ylabel('y (m)');legend('True','EKF+KF','EKF','KF','No Filter','SVSF+KF','SVSF'); hold off;


figure;  plot(t, x_r(3,:));hold all; plot(t, x_ekf_r(3,:));  plot(t, x_ekfonly(3,:));  plot(t, x_kfonly(3,:));  plot(t, x_nofilter(3,:)); plot(t, x_SVSF_r(3,:));plot(t, x_SVSFonly(3,:)); xlabel('Time (s)');ylabel('Angular Position (rad)');legend('True','EKF+KF','EKF','KF','No Filter','SVSF+KF','SVSF');hold off;


