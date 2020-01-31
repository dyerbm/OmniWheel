clc
clear all

dt=0.02; %polling rate of the vicon system (actually 50Hz?)
timeGlobal=0;
startTime=tic;

matcounter = 1; % Starting row for output matrix
max_operation = 40; % Maximum time robot will move
matrixsize = uint64((max_operation)/dt); % Based on the time for operation, will wait 1 second after robot stops to end recording
firstLine=0;

Sheet1Mat = zeros(matrixsize,11);

%marker positions
rb1=[0,0,0];
rb2=[0,0,0];
rb3=[0,0,0];
rb4=[0,0,0];
rb5=[0,0,0];

%info needed for controller

n=3; %num states
m=3;%num measurements
C=eye(m);

rR=0.195; %COM to wheel distance
M=3.4; %mass of the robot
IR=0.069/255; %robot moment of inertia

dx=[0;0;0]; %initialize desired values
dxdot=[0;0;0];
dxddot=[0;0;0];

x=[0;0;0];%initialize position vector
z=x;

Kp=[-200,0,50;0,200,50;200,0,50;0,-200,50]; % THESE BETTER HAVE NEGATIVE EIGSSSS
Kd=[0,0,0;0,0,0;0,0,0;0,0,0];

u=Kd*(dxdot-xdot)+Kp*(dx-x); %calculate controller based on position/velocity
if(max(abs(u))>255) %normalize controller
    u=u/max(abs(u))*255
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

PWMString = '0,0,0,0*';

fprintf(serialPortObj, PWMString);
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

    theta = atan(vector_ca(2)/vector_ca(1))-5*pi/4;
    
    xR = rb5(1);
    yR = rb5(2);
    thetaR = theta;

    %determine starting position of the robot
    if (firstLine == 0 )
        xRi = xR;
        yRi = yR;
        firstLine=1;
    
    else
    
        % Find xR, yR, thetaR - Real values

        xR = (xR-xRi);
        yR = (yR-yRi); 
        thetaR = theta;

        xold=x; %old position vector
        x=[xR/1000;yR/1000;thetaR]; %position vector in meters
        timeNow=toc(startTime);
        xdot=(x-xold)*(timeNow-timeGlobal); %velocity vector
        
        timeGlobal = timeNow;

        % Find xD, yD, thetaD - Desired values

        [dx, dxdot, dxddot] = desired_trajectory(timeGlobal);

        
        %calculate u and assign the observables
        y=C*x;
        u=Kd*(dxdot-xdot(:,k))+Kp*(dx-x(:,k)); %calculate controller based on position/velocity
        if(max(abs(u))>255) %normalize controller
            u=u/max(abs(u))*255
        end
        
        PWMString = sprintf('%.0f,%.0f,%.0f,%.0f*', u(1),u(2),u(3),u(4));
        fprintf(serialPortObj, PWMString);
        matcounter;
        
        

        %save it all to a sheet
        Sheet1Mat(matcounter,:) = [timeGlobal, xR,yR,thetaR, dx(1), dx(2), dx(3), PWM(1), PWM(4), PWM(3), PWM(2)]; % Gives raw data
        matcounter = matcounter + 1;


        sim(matcounter,:)=[timeGlobal,yd(1),yd(2),yd(3),simz1(1),simz1(2),simz1(3)];
    end
end % end of while loop, everything before this runs until the end of the script

figure
plot(sim(:,2),sim(:,3),sim(:,5),sim(:,6),Sheet1Mat(:,2),Sheet1Mat(:,3))
legend('desired path','simulated path','real path')
xlabel('x position (mm)')
ylabel('y position (mm)')

figure
plot(sim(:,2),sim(:,3),Sheet1Mat(:,2),Sheet1Mat(:,3))
legend('desired path','real path')
xlabel('x position (mm)')
ylabel('y position (mm)')

fprintf(serialPortObj, '0,0,0,0*');
pause(0.2)
fprintf(serialPortObj, '0,0,0,0*');

xlswrite(".\" + strcat('Controller_Output', '.xlsx'), Sheet1Mat);