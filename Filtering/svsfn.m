%% SVSF code (with a covariance derivation)
function [x, P, errk,q] = svsfn(x, z, err, u, vs, P, xNon, A, C, Q, R) % SVSF function for linear system and measurements example (note: you just need to change the state and measurement equations)
n = size(x,1); % Defines number of states
m = size(z,1); % Defines number of measurements
Psi = 1e0*[1; 1; 1; 1]; % Defines smoothing boundary layer widths (user defined)
% Psi = 10*[0.00001; 0.0001; .001; 1e-6]; % Defines smoothing boundary layer widths (user defined)
% Psi = 5*[0.013; 0.015; 0.0000096; 3e-3];  % Tuned
Gamma = 0.1*1e1; % Defines gamma (SVSF 'memory')
sat = zeros(m,1); % Initializes saturation terms
% Prediction Stage
x = xNon([x; u; vs]); % Predicts state estimates
P = A([x;u;vs])*P*A([x;u;vs])' + Q; % Predicts state error covariance
errk = z - C*x; % Predicts measurement error (innovation)
% Update Stage
for i = 1:m % For loop that calculates the saturated terms (error/Psi)
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
errk = z - C*x; % Updates measurement error (innovation)
q = (errk)'/(C*P*C' + R)*(errk);
end