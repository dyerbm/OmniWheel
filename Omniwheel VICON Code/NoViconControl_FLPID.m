clc

%% Initializes Parameters
timenow=0.01;
timeglobal=0;
tglobal=tic;

k = 1; % Starting row for output matrix
max_operation = 10; % Maximum time robot will move
matrixsize = max_operation * 50 + 50; % Based on the time for operation, will wait 1 second after robot stops to end recording
 
Sheet1Mat = zeros(matrixsize,14);

tf = max_operation; % Final time in simulation
T = 2e-2; % Sample rate
t = 0:T:tf; % Time vector
n = 3; % Number of states
m = 3; % Number of measurements
p = 4; % Number of Inputs
x = zeros(n, length(t)+10); % Initializes states to zero
x_d = x; %Initializes desired x states
z = zeros(m, length(t)+10); % Initializes measurements to zero
u = zeros(p, length(t)+10); % Initializes controller
v = zeros(3,length(t)+10); %initialize linearized controller
e=x;edot=x;eint=x; %initialize error terms

rr=0.195;

%A = [-cos(z(3:1)]; % System matrix
B = @(y)[(-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))-sin(y(3)))/(2*sqrt(2));
    (cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2));
    1/(4*rr), 1/(4*rr), 1/(4*rr), 1/(4*rr)]; % Input matrix
Binv = @(y)[-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    -((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr];

C = eye(m); % Measurement matrix

Kp=-[20,0,0; %Proportional Gain
    0,20,0;
    0,0,6]*50;
Kd=-[1,0,0; %Derivative Gain
    0,1,0;
    0,0,0.5]*50;
Ki=-[5,0,0; %Integral Gain
    0,5,0;
    0,0,1]*0.5;

for i=1:length(t)+10 %Fill in desired path
    x_d(:,i)=[0;0;0];
end


delete(instrfind);
port = 'COM3'; % Replace with whatever the USB serial bus from the XBee module is on (was 7)
serialPortObj = serial(port, 'BaudRate', 9600);
fopen(serialPortObj);

fprintf(serialPortObj,'0,0,0,0*');





while(timeglobal <= max_operation)

    % Getting angle relative to x axis.
    %vector_ca = [cos(pi/2) sin(pi/2); -sin(pi/2) cos(pi/2)]*[(rb4(1) - rb5(1))/1000; (rb4(2) - rb5(2))/1000]; %get position vector
    theta = 7*pi/4; %calculate angle

    % Find xR, yR, thetaR - Real values

    xR = 1; %get x position
    yR = 1; %get y position
    thetaR = theta; %get theta position
    
    z(:,k+1)=[xR;yR;thetaR];

    %CALCULATE CONTROLLER
    e(:,k) = z(:,k)-x_d(:,k); %calculate the error
    e(1,k)=1; %force theta
    e(2,k)=0;
    e(3,k)=0;
    e(:,k)
    
    if k~=1 %calculate edot if no on first time step
        edot(:,k) = (z(:,k)-z(:,k+1))/T; %calculate the derivative error term
    end
    numterms=100;
    if k>numterms %calculate integral error term once enough time has passed
        for i=k-numterms:k
            eint(:,k) = eint(:,k)+e(:,i);
        end
    end
    
    v(:,k+1)= Kp*e(:,k); %calculate linear P controller
    v(:,k+1) = Kp*e(:,k)+Kd*edot(:,k);%calculate linear PD controller
    v(:,k+1) = Kp*e(:,k)+Kd*edot(:,k)+Ki*eint(:,k);%calculate linear PID controller
    u(:,k+1)=Binv(z(:,k))*v(:,k+1);%calculate non linear controller
    
    %make max velocity 1
    if max(abs(u(:,k+1)))>100
       u(:,k+1)=u(:,k+1)/max(abs(u(:,k+1)))*100;
       k
    end
    u(:,k+1)=u(:,k+1);

    volts_to_send = sprintf('%d,%d,%d,%d*', int16(u(4,k+1)), int16(u(3,k+1)), int16(u(2,k+1)), int16(u(1,k+1)));
    volts_to_send
    fprintf(serialPortObj, volts_to_send);

    timeglobal = toc(tglobal);
    
    k = k + 1; %update step
    
end
    

volts_to_send='0,0,0,0*';
fprintf(serialPortObj,volts_to_send);
pause(0.2);
fprintf(serialPortObj,volts_to_send);
    
    
    
    
    