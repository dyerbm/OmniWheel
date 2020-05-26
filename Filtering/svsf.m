%% SVSF code (with a covariance derivation)
function [x, P, errk] = svsf(x, z, err, u, P, A, B, C, Q, R) % SVSF function for linear systems and measurements
n = size(x,1); % Defines number of states
m = size(z,1); % Defines number of measurements
Gamma = 0.1; % Defines SVSF memory term
Psi = 1e-3*[1; 1; 1; 1]; % Defines SVSF boundary layer terms
sat = zeros(m,1); % Iniitalizes saturation terms
% Prediction stage
x = A*x + B*u; % Predicts state estimates
P = A*P*A' + Q; % Predicts state error covariance
errk = z - C*x; % Predicts measurement errors (innovation)
% Update stage
for i = 1:m % For loop that calculates saturation terms (error/Psi)
    if abs(errk(i)/Psi(i)) >= 1
        sat(i) = sign(errk(i)/Psi(i));
    else
        sat(i) = errk(i)/Psi(i);
    end
end
E = abs(errk) + Gamma.*abs(err); % Calculates absolute error terms
K = (pinv(C)*diag(E.*sat))/diag(errk); % Calculates SVSF gain
x = x + K*(z - C*x); % Updates state estimates
P = (eye(n) - K*C)*P*(eye(n) - K*C)' + K*R*K'; % Updates state error covariance
errk = z - C*x; % Updates measurement errors (innovation)
end