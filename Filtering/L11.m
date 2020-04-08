%% Air Traffic Control Example for ENGG*6090*01 W20 by S. A. Gadsden (gadsden@uoguelph.ca)
% Setting up Matlab
clear; % Clears workspace
close all; % Closes all open figures
clc; % Clears screen
%% Definitions and Initializations
tf = 500; % Final simulation time
T = 5; % Sample rate (of radar)
t = 0:T:tf; % Sets up the time vector
n = 5; % Number of states
m = 2; % Number of measurements
x = [25e3 -120 10e3 0 0]'; % Initial state values
x_kf = x; % Initializes the KF estimates
x_ekf = x_kf; % Initializes the EKF estimates
x_ukf = x_kf; % Iniitalizes the UKF estimates
z = [x(1); x(3)]; % Initializes the measurements
H = [1 0 0 0 0; % Defines the measurement matrix
    0 0 1 0 0];
Hlin = H; % Defines the linearized measurement matrix (which is just H above)
UM = [1 T 0 0 0; % Defines the uniform motion model used by the KF
    0 1 0 0 0;
    0 0 1 T 0;
    0 0 0 1 0;
    0 0 0 0 0];
CT = @(x) [1 sin(x*T)/x 0 -(1-cos(x*T))/x 0; % Defines a function for the coordinated turn model
    0 cos(x*T) 0 -sin(x*T) 0;
    0 (1-cos(x*T))/x 1 sin(x*T)/x 0;
    0 sin(x*T) 0 cos(x*T) 0;
    0 0 0 0 1];
CTlin = @(x) [1 sin(x(5)*T)/x(5) 0 -(1-cos(x(5)*T))/x(5) cos(x(5)*T)*T*x(2)/x(5)-sin(x(5)*T)*x(2)/x(5)^2-sin(x(5)*T)*T*x(4)/x(5)-(-1+cos(x(5)*T))*x(4)/x(5)^2; % Defines the linearized coordinated turn model
    0 cos(x(5)*T) 0 -sin(x(5)*T) -sin(x(5)*T)*T*x(2)-cos(x(5)*T)*T*x(4);
    0 (1-cos(x(5)*T))/x(5) 1 sin(x(5)*T)/x(5) sin(x(5)*T)*T*x(2)/x(5)-(1-cos(x(5)*T))*x(2)/x(5)^2+cos(x(5)*T)*T*x(4)/x(5)-sin(x(5)*T)*x(4)/x(5)^2;
    0 sin(x(5)*T) 0 cos(x(5)*T) cos(x(5)*T)*T*x(2)-sin(x(5)*T)*T*x(4);
    0 0 0 0 1];
CT0 = @(x) [1 T 0 0 -0.5*T^2*x(4); % Linearized CT model when omega is very, very small (prevents simulation from 'blowing up' aka numerical instability)
    0 1 0 0 -T*x(4);
    0 0 1 T 0.5*T^2*x(2);
    0 0 0 1 T*x(2);
    0 0 0 0 1];
L1 = 0.16; % Power spectral parameter 1 (definition)
L2 = 0.01; % Power spectral parameter 2 (definition)
Q1 = L1*[T^3/3 T^2/2 0 0 0; % System noise covariance for UM model
    T^2/2 T 0 0 0;
    0 0 T^3/3 T^2/2 0;
    0 0 T^2/2 T 0;
    0 0 0 0 1e-12];
Q2 = L1*[T^3/3 T^2/2 0 0 0; % System noise covariance for CT model
    T^2/2 T 0 0 0;
    0 0 T^3/3 T^2/2 0;
    0 0 T^2/2 T 0;
    0 0 0 0 T*L2/L1];
R = (500)^2*eye(m); % Measurement noise covariance
P_kf = [R(1,1) 0 0 0 0; % Initializes the state error covariance for KF
    0 100 0 0 0;
    0 0 R(2,2) 0 0;
    0 0 0 100 0;
    0 0 0 0 1];
P_ekf = P_kf; % Initializes the state error covariance for the EKF
P_ukf = P_kf; % Initializes the state error covariance for the UKF
w = mvnrnd(zeros(tf/T+1,n),Q1)'; % Generates the system noise
v = mvnrnd(zeros(tf/T+1,m),R)'; % Generates the measurement noise
%% Simulate Dynamics and Run Estimation Strategies
for k = 1:tf/T % Generates the true state trajectories and measurements
    if k <= 125/T % For the first 125 seconds, the aircraft uses the UM model
        x(:,k+1) = UM*x(:,k) + w(:,k); % Calculates the true states
    end
    if k > 125/T && k <= 215/T % For the next 90 seconds, the aircraft uses the CT model at +1 deg/sec
        x(:,k+1) = CT(pi/180)*x(:,k) + w(:,k); % Calculates the true states
        if k==130/T
           x(5,k+1)=pi/180
        end
    end
    if k > 215/T && k <= 340/T % For the next 125 seconds, the aircraft uses the UM model
        x(:,k+1) = UM*x(:,k) + w(:,k); % Calculates the true states
    end
    if k > 340/T && k <= 370/T % For the next 30 seconds, the aircraft uses the CT model at -3 deg/sec
        x(:,k+1) = CT(-3*pi/180)*x(:,k) + w(:,k); % Calculates the true states
    end
    if k > 370/T % For the rest of the simulation, the aircraft uses the UM model
        x(:,k+1) = UM*x(:,k) + w(:,k); % Calculates the true states
        if k==375/T
           x(5,k+1)=3*pi/180
        end
    end
    z(:,k+1) = H*x(:,k+1) + v(:,k+1); % Generates the measurements
    x(:,k+1);
end
%z(:,50)=z(:,50)*500;
for k = 1:tf/T % Applying KF and EKF Estimation Strategies
    [x_kf(:,k+1), P_kf(:,:,k+1)] = kf(x_kf(:,k), z(:,k+1), 0, P_kf(:,:,k), UM, 0, H, Q1, R); % Calls the KF Function to calculate the KF estimates and state error covariance
    [x_ekf(:,k+1), P_ekf(:,:,k+1)] = ekf_atc(x_ekf(:,k), z(:,k+1), 0, P_ekf(:,:,k), CT, CTlin, CT0, 0, H, Hlin, Q2, R); % Calls the EKF Function to calculate the EKF estimates and state error covariance
    [x_ukf(:,k+1), P_ukf(:,:,k+1)] = ukf_atc(x_ukf(:,k), z(:,k+1), P_ukf(:,:,k), CT, CT0, H, Q2, R); % Calls the UKF Function to calculate the EKF estimates and state error covariance
end
%% Generates RMSE and Plots
% RMSE calculations (another way to do it!)
rmse_kf = zeros(n,1);
rmse_ekf = zeros(n,1);
rmse_ukf = zeros(n,1);
for i = 1:n
    rmse_kf(i) = sqrt(sum((x(i,:)-x_kf(i,:)).^2)/length(t)); % Calculates KF RMSE
    rmse_ekf(i) = sqrt(sum((x(i,:)-x_ekf(i,:)).^2)/length(t)); % Calculates EKF RMSE
    rmse_ukf(i) = sqrt(sum((x(i,:)-x_ukf(i,:)).^2)/length(t)); % Calculates UKF RMSE
end
State = ['1'; '2'; '3'; '4'; '5']; % Creates label for table of RMSE results
Results_RMSE = table(State,rmse_kf,rmse_ekf,rmse_ukf); % Presents the RMSE results in the form of a table
Results_RMSE.Properties.VariableNames = {'State','KF','EKF','UKF'}; % Labels the columns
fprintf('Simulation Results: \n\n' ); % Displays title for results
disp(Results_RMSE); % Displays RMSE results table
% % Creates plot of true trajectory and measurements
% figure; plot(x(1,:),x(3,:)); hold all; plot(z(1,:),z(2,:),'rx');
% xlabel('x (m)'); ylabel('y (m)'); title('ATC Problem'); legend('Trajectory','Radar Measurement','Location','SouthEast'); xlim([-20e3 30e3]); ylim([-25e3 15e3]);
% Creates plot of estimated position
figure; plot(x(1,:),x(3,:)); hold all; plot(x_kf(1,:),x_kf(3,:),'-gx'); plot(x_ekf(1,:),x_ekf(3,:),'--bx'); plot(x_ukf(1,:),x_ukf(3,:),'--rx');
xlabel('x (m)'); ylabel('y (m)'); title('ATC Estimation Results'); legend('True','KF (UM)','EKF (CT)','UKF (CT)','Location','SouthEast');

% Creates plot of estimated velocities
figure; plot(t,x(4,:)); hold all; plot(t,x_kf(4,:),'-gx'); plot(t,x_ekf(4,:),'--bx'); plot(t,x_ukf(4,:),'--rx');
xlabel('t'); ylabel('v_y (m/s)'); title('ATC Estimation Results'); legend('True','KF (UM)','EKF (CT)','UKF (CT)','Location','SouthEast');

figure; plot(t,x(2,:)); hold all; plot(t,x_kf(2,:),'-gx'); plot(t,x_ekf(2,:),'--bx'); plot(t,x_ukf(2,:),'--rx');
xlabel('t'); ylabel('v_x (m/s)'); title('ATC Estimation Results'); legend('True','KF (UM)','EKF (CT)','UKF (CT)','Location','SouthEast');


% Creates plot of estimated turn rates
figure; plot(t,x(5,:)); hold all; plot(t,x_kf(5,:),'-gx'); plot(t,x_ekf(5,:),'--bx'); plot(t,x_ukf(5,:),'--rx');
xlabel('time'); ylabel('turn rate (rad/s)'); title('ATC Estimation Results'); legend('True','KF (UM)','EKF (CT)','UKF (CT)','Location','SouthEast');

% % Creates plot of position tracking errors for KF and EKF estimates
% figure; plot(t,sqrt((x(1,:)-x_kf(1,:)).^2+(x(3,:)-x_kf(3,:)).^2)); hold all; plot(t,sqrt((x(1,:)-x_ekf(1,:)).^2+(x(3,:)-x_ekf(3,:)).^2),'--'); plot(t,sqrt((x(1,:)-x_ukf(1,:)).^2+(x(3,:)-x_ukf(3,:)).^2),'--');
% xlabel('Time (sec)'); ylabel('Position Tracking Error (m)'); title('Position Tracking Error'); legend('KF (UM)','EKF (CT)','UKF (CT)','Location','NorthWest');