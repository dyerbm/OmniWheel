%% KF function
function [x, P] = kf(x, z, u, P, A, B, C, Q, R)
n = size(x,1); % Defines number of states
% Prediction stage
x = A*x + B*u; % Predicts state estimates
P = A*P*A' + Q; % Predicts state error covariance
% Update stage
K = P*C'*pinv(C*P*C' + R); % Calculates Kalman gain
x = x + K*(z - C*x); % Updates state estimates
P = (eye(n) - K*C)*P*(eye(n) - K*C)' + K*R*K'; % Updates state error covariance
end