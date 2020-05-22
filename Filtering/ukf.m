%% Unscented Kalman Filter (UKF) Code [nonlinear system, linear measurement]
function [x, P] = ukf(x, z, u, vs, P, xNon, C, Q, R)
% a) Iniitalize stage
n = size(x,1); % Defines number of states
m = size(z,1); % Defines number of measurements
kappa = 1e-3; % Defines kappa value (user defined)
sqrtnkp = sqrtm((n+kappa)*P); % Calculates square root of (n+kappa)*P
X = zeros(n,2*n+1); % Initializes sigma points to zero
W = zeros(1,2*n+1); % Initializes weights to zero for each sigma point
% Sigma point #1
X(:,1) = x; % Defines first sigma point
W(1) = kappa/(n+kappa); % Defines weight for first sigma point
% Sigma points 2 to 2*n+1
for i = 1:n
    X(:,i+1) = x + sqrtnkp(:,i); % Defines 2 to n+1 sigma points
    W(i+1) = 1/(2*(n+kappa)); % Defines corresponding weights
    X(:,i+n+1) = x - sqrtnkp(:,i); % Defines n+2 to 2*n+1 sigma points
    W(i+n+1) = 1/(2*(n+kappa)); % Defines corresponding weights
end
% b) Prediction stage
for i = 1:2*n+1
    X(:,i) = xNon([X(:,i);vs; u]); % Calculates predicted sigma points
end
x = X*W'; % Predicted state estimates
P = Q; % Starts predicted state error covariance calculation
for i = 1:2*n+1 % For loop that calculates predicted state error covariance
    P = P + W(i)*(X(:,i)-x)*transpose(X(:,i)-x);
end
Z = C*X; % Calculates measurements based on sigma points
zk = Z*W'; % Calculates predicted measurements
% c) Update stage
Pzz = R; % Starts predicted state error covariance calculation
for i = 1:2*n+1 % For loop that calculates predicted innovation covariance
    Pzz = Pzz + W(i)*(Z(:,i)-zk)*transpose(Z(:,i)-zk);
end
Pxz = zeros(n,m); % Starts predicted cross-covariance
for i = 1:2*n+1 % For loop that calculates predicted cross-covariance
    Pxz = Pxz + W(i)*(X(:,i)-x)*transpose(Z(:,i)-zk);
end
K = Pxz/Pzz; % Calculates UKF gain
x = x + K*(z-zk); % Updates state estimates
P = P - K*Pzz*transpose(K); % Updates state error covariance
end