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
x = zeros(n, length(t)); % Initializes states to zero
x_d = x; %Initializes desired x states
z = zeros(m, length(t)); % Initializes measurements to zero
u = zeros(p, length(t)); % Initializes controller
v = zeros(3,length(t)); %initialize linearized controller

%A = [-cos(z(3:1)]; % System matrix
B = @(y)[(-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))-sin(y(3)))/(2*sqrt(2));
    (cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))-sin(y(3)))/(2*sqrt(2)), (-cos(y(3))+sin(y(3)))/(2*sqrt(2)), (cos(y(3))+sin(y(3)))/(2*sqrt(2));
    1/(4*rr), 1/(4*rr), 1/(4*rr), 1/(4*rr)]; % Input matrix
Binv = @(y)[-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    -((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),-((cos(y(3))-sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr;
    ((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)),((cos(y(3))+sin(y(3)))*sqrt(2))/(2*(cos(y(3))^2+sin(y(3))^2)), rr];

C = eye(m); % Measurement matrix

Kp=-[5,0,0; %Proportional Gain
    0,5,0;
    0,0,2]*0.1;
Kd=-[1,0,0; %Derivative Gain
    0,1,0;
    0,0,0.5]*0.1;
Ki=-[1,0,0; %Integral Gain
    0,1,0;
    0,0,1];

for k=1:length(t) %Fill in desired path
    x_d(:,k)=[0;0;0];
end

%% select serial port.
delete(instrfind);
port = 'COM7'; % Replace with whatever the USB serial bus from the XBee module is on (was 7)
serialPortObj = serial(port, 'BaudRate', 9600);
fopen(serialPortObj);

%% CONNECT TO DATA STREAM
% Load the SDK
Client.LoadViconDataStreamSDK();
fprintf( 'Vicon Data Stream SDK loaded\n' );

% Connect to a server
HostName = 'localhost:801';
MyClient = Client(); % Client obj
fprintf( 'Connecting to %s ...\n', HostName );
while ~MyClient.IsConnected().Connected
  MyClient.Connect( HostName );
end

% Enable some different data types
MyClient.EnableSegmentData();
MyClient.EnableMarkerData();

% Set the streaming mode
MyClient.SetStreamMode( StreamMode.ClientPull );

% Set the axis mapping
MyClient.SetAxisMapping( Direction.Forward, ...
                         Direction.Left,    ...
                         Direction.Up );    % Z-up
                                                  
tempcount=0;      
counter=0;
flag1=0;
DATACORRECTION=0;       
data=[1000,1000];

volts_to_send = "0,0,0,0*";

fprintf(serialPortObj, volts_to_send);
tglobal=tic;


%% GET DATA

while(timeglobal <= max_operation)
    
    % 1 - Get a frame
    while MyClient.GetFrame().Result.Value ~= Result.Success
    end
    
    % Print the frame number
    Output_GetFrameNumber = MyClient.GetFrameNumber();
    
    % 2 - Get the subject
    SubjectCount = MyClient.GetSubjectCount().SubjectCount;

    for SubjectIndex = 1:SubjectCount
        timer=tic;
        SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
        
        % 3 - get origin data
        if strcmpi(SubjectName, 'origin')
            % Get translation data of origin
            MarkerCount = MyClient.GetMarkerCount( SubjectName ).MarkerCount;

            for MarkerIndex = 1:MarkerCount
                MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;
                
                if MarkerName ~= ' ' % error check - it is labelled
                    MarkerTranslation = MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );
                    
                   if strcmpi(MarkerName, 'origin')
                        origin = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                   end
                   
                end
                
            end
            
        end % end of get origin data
        
        % 3 - get robot data
        if strcmpi(SubjectName, 'robot')
            % Get translation data of all 4 markers
            MarkerCount = MyClient.GetMarkerCount( SubjectName ).MarkerCount;
%             fprintf( '    Markers (%d):\n', MarkerCount );
            for MarkerIndex = 1:MarkerCount
                MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;
                
                if MarkerName ~= ' ' % error check - it is labelled

                    MarkerTranslation = MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );
                    
                    if strcmpi(MarkerName, 'rb5')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb5 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                        end
                        
                    end
                    if strcmpi(MarkerName, 'rb1')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb1 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                        end
                        
                    end
                    if strcmpi(MarkerName, 'rb2')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb2 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                        end
                        
                    end
                    if strcmpi(MarkerName, 'rb3')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb3 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                        end
                        
                    end
                    if strcmpi(MarkerName, 'rb4')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb4 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                        end
                        
                    end
                    
                end % end of if for checking to make sure the marker is valid
                
            end % end of for loop for checking markers
            
        end % end of robot segment
    end
        
%% Processing the data in real time

    % Getting angle relative to x axis.
    vector_ca = [(rb4(1) - rb5(1))/1000, (rb4(2) - rb5(2))/1000]; %get position vector

    theta = atan(vector_ca(2)/vector_ca(1)); %calculate angle

    % Find xR, yR, thetaR - Real values

    xR = rb5(1)/1000; %get x position
    yR = rb5(2)/1000; %get y position
    thetaR = theta; %get theta position
    
    x(:,k)=[xR;yR;thetaR]

    %u(:,k+1) = [sin(pi*k*T);cos(pi*k*T);-sin(pi*k*T);-cos(pi*k*T)];% calculate controller
    e = z(:,k)-x_d(:,k); %calculate the error
    edot=[0;0;0]; %set edot
    %eint = ;%calculate the integral error term
    
    if k~=1 %calculate edot if no on first time step
        edot = (z(:,k-1)-z(:,k))/T; %calculate the derivative error term
    end
    
    v(:,k+1) = Kp*e+Kd*edot;%calculate linear controller
    u(:,k+1)=Binv(z(:,k))*v(:,k+1);%calculate non linear controller
    
    %limit maximum velocity
    if max(abs(B(x(:,k))*u(:,k+1)/T))>1
       u(:,k+1)=u(:,k+1)/max(abs(B(x(:,k))*u(:,k+1)));
       k
    end
    
    
    x(:,k+1) = x(:,k) + B(x(:,k))*u(:,k+1)*T; % Linear state space equation
    z(:,k+1) = C*x(:,k+1); % Linear measurement equation

    [u', err', integrator_error']
    volts_to_send = sprintf('%d.5,%d.5,%d.5,%d.5*', u(1), u(2), u(3), u(4));
    fprintf(serialPortObj, volts_to_send);

    Sheet1Mat(k,:) = [timeglobal, xR, yR, thetaR, xD, yD, thetaD, xE, yE, thetaE, u(1), u(2), u(3), u(4)]; % Gives raw data

    k = k + 1;

    timeglobal = toc(tglobal);

end % end of while loop, everything before this runs until the end of the script

fprintf(serialPortObj, '0,0,0,0*');

xlswrite("C:\Users\Vicon\Desktop\Biglar Begian VICON\" + strcat('Controller_Output', '.xlsx'), Sheet1Mat);