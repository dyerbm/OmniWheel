%% EKF function (for ATC)
function [x, P] = ekf_atc(x, z, u, P, xNon, Alin, A0, B, C, Clin, Q, R)
n = size(x,1); % Defines number of states
% Prediction stage
if abs(x(5)) < 1e-5 % Checks for very small turn rates
    x = A0(x)*x + B*u; % Modified CT model for small turn rates
    P = A0(x)*P*A0(x)' + Q; % Modified covariance model for small turn rates
else % This is normal CT motion (not small turn rates)
    x = xNon(x(5))*x + B*u; % Predicts state estimates
    P = Alin(x)*P*Alin(x)' + Q; % Predicts state error covariance
end
% Update stage
K = P*Clin'*pinv(Clin*P*Clin' + R); % Calculates Kalman gain
% K = P*C'*pinv(Clin(x)*P*Clin(x)' + R); % Calculates Kalman gain
x = x + K*(z - C*x); % Updates state estimates
% x = x + K*(z - zNon(x)); % Updates state estimates
P = (eye(n) - K*Clin)*P*(eye(n) - K*Clin)' + K*R*K'; % Updates state error covariance
% P = (eye(n) - K*Clin(x))*P*(eye(n) - K*Clin(x))' + K*R*K'; % Updates state error covariance
end