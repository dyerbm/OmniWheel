dt=0.01; %polling rate of the vicon system (actually 50Hz?)
timeGlobal=0;
startTime=tic;

matcounter = 1; % Starting row for output matrix
max_operation = 10; % Maximum time robot will move
matrixsize = uint64((max_operation+1)/dt); % Based on the time for operation, will wait 1 second after robot stops to end recording

Sheet1Mat = zeros(matrixsize,11);

%marker positions
rb1=[0,0,0];
rb2=[0,0,0];
rb3=[0,0,0];
rb4=[0,0,0];
rb5=[0,0,0];

%info needed for controller
y=[0;0;0];yd=[0;0;0];dyd=[0;0;0];ddyd=[0;0;0]; %will redefine these

z2=[0;0;0]; %velocities
z1=[0;0;0]; %position

Iw=0.0000427; %wheel moment of inertia - FIX THIS VALUE
rw=0.034925; %wheel radius
rR=0.195; %COM to wheel distance
m=3.5; %mass of the robot
IR=0.05; %robot moment of inertia

%define M/N dependancies
I=eye(4);
S=[0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0];
M=[m,0,0;0,m,0;0,0,IR];
Q=[-sin(z1(3)), -sin(z1(3)+pi/2), -sin(z1(3)+pi), -sin(z1(3)+3*pi/2);
    cos(z1(3)), cos(z1(3)+pi/2), cos(z1(3)+pi), cos(z1(3)+3*pi/2);
    rR,rR,rR,rR];
R=[0,1,rw;-1,0,rw;0,-1,rw;1,0,rw];
T=[cos(z1(3)),sin(z1(3)),0;-sin(z1(3)),cos(z1(3)),0;0,0,1];
Tdot=z2(3)*[-sin(z1(3)),cos(z1(3)),0;-cos(z1(3)),-sin(z1(3)),0;0,0,0];

%define M and N
Nstar=Iw/rw*(S+I)*R*Tdot;
Mstar=rw*Q\M+Iw/rw*(S+I)*R*T;

%define control matrices
k1=[-4.1,0,0;0,-4.1,0;0,0,-4.1]; % THESE BETTER HAVE NEGATIVE EIGSSSS
k2=[-5,0,0;0,-5,0;0,0,-5];

%define controller and torque
u=-k1*(dyd-z2)-k2*(yd-z1);
tau=Mstar*(ddyd+u)+Nstar*z1;


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

while(timeGlobal <= max_operation)
    
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
    vector_ca = [(rb4(1) - rb5(1))/1000, (rb4(2) - rb5(2))/1000];

    theta = atan(vector_ca(2)/vector_ca(1));

    %determine starting position of the robot
    if (xR~=0 && yR~=0 && firstLine == 0 )
        xRi = xR;
        yRi = yR;
        firstLine=1;
    
    elseif (firstLine~=0)
    
        % Find xR, yR, thetaR - Real values

        xD = xD+xRi;
        yD = yD+yRi;
        thetaR = theta;

        z1old=z1; %old position vector
        z1=[xR;yR;thetaR]; %position vector
        z2=(z1-z1old)*dt; %velocity vector

        % Find xD, yD, thetaD - Desired values

        [yd, dyd, ddyd] = desired_trajectory(timeGlobal);

        % Recalculate dynamics dependancies

        Q=[-sin(z1(3)), -sin(z1(3)+pi/2), -sin(z1(3)+pi), -sin(z1(3)+3*pi/2);
        cos(z1(3)), cos(z1(3)+pi/2), cos(z1(3)+pi), cos(z1(3)+3*pi/2);
        rR,rR,rR,rR];
        T=[cos(z1(3)),sin(z1(3)),0;-sin(z1(3)),cos(z1(3)),0;0,0,1];
        Tdot=z2(3)*[-sin(z1(3)),cos(z1(3)),0;-cos(z1(3)),-sin(z1(3)),0;0,0,0];

        Nstar=Iw/rw*(S+I)*R*Tdot;
        Mstar=rw*pinv(Q)*M+Iw/rw*(S+I)*R*T;
        
        %calculate u and assign the observables
        y=z1;
        u=-k1*(dyd-z2)-k2*(yd-z1);
        
        %calculate torque
        tau=Mstar*(ddyd+u)+Nstar*z1

        %normalize the torques
        if max(abs(tau))>23.144 %max torque of 23.144 Nm
            tau = tau./max(abs(tau))*23.144;
        end
        
        %create  and send PWM values based on torques
        PWM = int16(tau./23.144*255); 
        PWMString = sprintf('%d,%d,%d,%d*', PWM(1), PWM(2), PWM(3), PWM(4));
        fprintf(serialPortObj, PWMString);

        %save it all to a sheet
        Sheet1Mat(matcounter,:) = [timeGlobal, z1(1),z1(2),z1(3), yd(1),yd(2),yd(3), PWM(1), PWM(2), PWM(3), PWM(4)]; % Gives raw data
        matcounter = matcounter + 1;

        timeGlobal = toc(startTime);
    end
end % end of while loop, everything before this runs until the end of the script

fprintf(serialPortObj, '0,0,0,0*');

xlswrite("C:\Users\Vicon\Desktop\Biglar Begian VICON\" + strcat('Controller_Output', '.xlsx'), Sheet1Mat);