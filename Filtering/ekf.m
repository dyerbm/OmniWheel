%% EKF function
function [x, P] = ekf(x, z, u, P, xNon, Alin, C, Q, R)
n = size(x,1); % Defines number of states
% Prediction stage
x = xNon([x; u]); % Predicts state estimates
P = Alin(x)*P*Alin(x)' + Q; % Predicts state error covariance
% Update stage
K = P*C'*pinv(C*P*C' + R); % Calculates Kalman gain
% K = P*C'*pinv(Clin(x)*P*Clin(x)' + R); % Calculates Kalman gain
x = x + K*(z - C*x); % Updates state estimates
% x = x + K*(z - zNon(x)); % Updates state estimates
P = (eye(n) - K*C)*P*(eye(n) - K*C)' + K*R*K'; % Updates state error covariance
% P = (eye(n) - K*Clin(x))*P*(eye(n) - K*Clin(x))' + K*R*K'; % Updates state error covariance
end