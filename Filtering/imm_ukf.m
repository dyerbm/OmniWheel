function [XHAT, PHAT, MU] = imm_ukf(z, u, A1, A2, B, C, Q, R, check, Alin, A0, Clin) % IMM function
%% 0. Definitions and initializations
n = size(Q,1); % Defines number of states
num_sys = 2; % Defines number of systems (for this case)
persistent xhatk Phatk mu; % Keeps values temporarily stored for next iteration
p_ij = [0.90 0.10; 0.10 0.90]; % Defines mode transition matrix (user defined)
if check == 1
    xhat = [z(1);0;z(2);0;0]; % Defines estimate vector to measurements
    Phat = 10*Q; % Defines state error covariance matrix
    xhatk = [xhat xhat];  % Defines vector for mode-matched estimates
    Phatk(:,:,1) = Phat; % Defines matrix for mode-matched covariance's
    Phatk(:,:,2) = Phat; % Defines matrix for mode-matched covariance's
    mu = [0.90; 0.10]; % Initializes mode probabilities (user defined)
end
%% 1. Calculating mixing probabilities
mix = zeros(num_sys, num_sys); % Initializes mixing probabilities
cbar = p_ij'*mu; % Defines normalized constant (cbar)
for j = 1:num_sys
    for i = 1:num_sys
        mix(i,j) = p_ij(i,j)*mu(i)/cbar(j); % Calculates mixing probabilties
    end
end
%% 2. Interacting/mixing stage
xMix = zeros(n, num_sys); % Iniitalizes mode-matched state estimates to zero
PMix = zeros(n, n, num_sys); % Initializes mode-matched covariance's to zero
for j = 1:num_sys
    for i = 1:num_sys
        xMix(:,j) = xMix(:,j) + mix(i,j)*xhatk(:,i); % Calculates mode-matched state estimates
    end
    for i = 1:num_sys
        PMix(:,:,j) = PMix(:,:,j) + mix(i,j)*(Phatk(:,:,i) + (xhatk(:,i) - xMix(:,j))*(xhatk(:,i) - xMix(:,j))'); % Calculates mode-matched state estimates
    end
end
%% 3. Mode-matched filtering
[xhatk(:,1), Phatk(:,:,1), likelihood(1,1)] = kf_imm(xMix(:,1), z, u, PMix(:,:,1), A1, B, C, Q, R); % Calls the KF using A1
[xhatk(:,2), Phatk(:,:,2), likelihood(2,1)] = ukf_atc_imm(xMix(:,2), z, PMix(:,:,2), A2, A0, C, Q, R); % Calls the KF using A2

%% 4. Mode probability update stage
mu = likelihood.*cbar; % Calculates mode probability using LH values from KFs
mu = mu/sum(mu) % Normalizes mode probabilities
MU = mu; % Defines MU output for imm.m function
%% 5. Overall state estimates and covariance output
XHAT = zeros(n,1); % Initializes output state estimates
PHAT = zeros(n,n); % Initializes output state error covariance's
for i = 1:num_sys
    XHAT = XHAT + xhatk(:,i)*mu(i); % Calculates output state estimates
end
for i = 1:num_sys
    PHAT = PHAT + mu(i)*(Phatk(:,:,i) + (xhatk(:,i) - XHAT)*(xhatk(:,i) - XHAT)'); % Calculates output state error covariance's
end
end