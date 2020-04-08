%% KF function for IMM (L10)
function [x, P, LH] = kf_imm(x, z, u, P, A, B, C, Q, R)
n = size(x,1); % Defines number of states
% Prediction stage
x = A*x + B*u; % Predicts state estimates
P = A*P*A' + Q; % Predicts state error covariance
% Update stage
S = C*P*C' + R; % Calculates innovation covariance
K = P*C'*pinv(S); % Calculates Kalman gain
LH = 1/sqrt(abs(det(2*pi*S)))*exp(-0.5*(z-C*x)'*pinv(S)*(z-C*x)); % Calculates the likelihood value
if abs(LH) < 1e-40
    LH = 1e-40 % Prevents very, very small LH (numerical issues)
end
x = x + K*(z - C*x); % Updates state estimates
P = (eye(n) - K*C)*P*(eye(n) - K*C)' + K*R*K'; % Updates state error covariance
end