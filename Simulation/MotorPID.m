    
function [x, z] = MotorPID(x, z, x_d, tf, A_m, B_m, C_m)
% %% Initializing parameters
%     T = 2e-5; % Sample rate
%     t = 0:T:tf; % Time vector
% 
%     x_m = zeros(size(x,1), length(t)); % Initializes states to zero
%     z_m = zeros(size(z,1), length(t)); % Initializes measurements to zero
%     u_m = z_m; % Random input
%     e_m= z_m;
%     e_d_m=0;
%     eint_m=0;
%     x_d
% 
%     K_p=-10;
%     K_d=-1;
%     K_i=0;
% 
%     %% Set initial conditions
% 
%     x_m(:,1)=x; %any initial condition
%     z_m(:,1)=z; %set first measurement
% 
% 
%     %% Simulate dynamics (for loop)
%     for k = 1:length(t)-1 % For loop that simulates 1 second
% 
%         e_m(:,k)=z_m(:,k)-x_d;
%         e_m(:,k);
%         eint_m=sum(e_m,2)*T;
%         if k>2/T %calculate integral error term over previous 2 seconds
%             eint_m=sum(e_m(:,k-2/T:k),2);
%         end
%         if k~=1
%            e_d_m = (e_m(:,k)-e_m(:,k-1))/T;
%         end
%         u_m(:,k+1)=K_p*e_m(:,k)+K_d*e_d_m+K_i*eint_m;%calculate voltages for each motor
%         if abs(max(u_m(:,k+1)))>12
%            u_m(:,k+1)= u_m(:,k+1)/abs(max(u_m(:,k+1)))*12;
%         end
%         u_m(:,k+1);
% 
%         x_m(:,k+1)=A_m*x_m(:,k)+B_m*u_m(:,k+1);
%         z_m(:,k+1)=C_m*x_m(:,k+1);
%         z_m(:,k+1);
%     end
%     z;
%     x = x_m(:,length(x_m));
%     z = z_m(:,length(z_m));
%     z;
%      z = x_d;


%% plz work better

tf = 0.002; % Final time in simulation
T = 2e-5; % Sample rate
t = 0:T:tf; % Time vector

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
C_c=[1 0];
A_d=A_c*T+eye(n_m); %discrete system matrix
B_d=B_c*T; %Discrete input matrix
C_d=C_c; %Discrete Measurement Matrix
A_m=blkdiag(A_d,A_d,A_d,A_d);
B_m=blkdiag(B_d,B_d,B_d,B_d);
C_m=blkdiag(C_d,C_d,C_d,C_d); %Measurement Matrix

x_m = zeros(n_m*num_m, length(t)); % Initializes states to zero
z_m = zeros(m_m*num_m, length(t)); % Initializes measurements to zero
u_m = randn(p_m*num_m, length(t)); % Random input
e_m= z_m;
e_d_m=0;
eint_m=0;

K_p=-10;
K_d=0;
K_i=0;


x_m(:,1)=x; %any initial condition
%x(:,1)=x_d(:,1); %start at the proper position

z_m(:,1)=z; %set first measurement


%% Simulate dynamics (for loop)
for k = 1:length(t)-1 % For loop that simulates 1 second
    
    
    e_m(:,k)=z_m(:,k)-x_d;
    eint_m=sum(e_m,2)*T;
    if k>2/T %calculate integral error term over previous 2 seconds
        eint_m=sum(e_m(:,k-2/T:k),2);
    end
    if k~=1
       e_d_m = (e_m(:,k)-e_m(:,k-1))/T;
    end
    u_m(:,k+1)=K_p*e_m(:,k)+K_d*e_d_m+K_i*eint_m;%calculate voltages for each motor
    if abs(max(u_m(:,k+1)))>12
       u_m(:,k+1)= u_m(:,k+1)/abs(max(u_m(:,k+1)))*12;
       u_m(:,k+1);
    end
    
%     if k<length(t)/2
%         u_m(:,k+1)=[12;12;12;12]; %stabilize to the origin
%     else
%         u_m(:,k+1)=[0;0;0;0];
%     end
%       if k>length(t)/2
%         u_m(:,k+1)=[0;0;0;0];
%       end
    
    e_m(:,k);
    
    x_m(:,k+1)=A_m*x_m(:,k)+B_m*u_m(:,k+1);
    z_m(:,k+1)=C_m*x_m(:,k+1);
    
    x_m(:,k+1);
end
x=x_m(:,k+1);
z=z_m(:,k+1);
 end