timenow=0.01;
timeglobal=0;
tglobal=tic;

matcounter = 1; % Starting row for output matrix
max_operation = 20; % Maximum time robot will move
matrixsize = max_operation * 100 + 100; % Based on the time for operation, will wait 1 second after robot stops to end recording
firstLine = 0; %for zeroing position of robot
 
Sheet1Mat = zeros(matrixsize,14);

rb1=[0,0,0];
rb2=[0,0,0];
rb3=[0,0,0];
rb4=[0,0,0];
rb5=[0,0,0];

int_xE = 0;
int_yE = 0;
int_thetaE = 0;

integrator_error = [int_xE; int_yE; int_thetaE];

%% select serial port.
delete(instrfind);
port = 'COM7'; % Replace with whatever the USB serial bus from the XBee module is on (was 7)
serialPortObj = serial(port, 'BaudRate', 9600);
fopen(serialPortObj);

%% make data out file
prompt={'Please enter the name of the desired notebook'};
title='Excel notebook name';
notebook_name = inputdlg(prompt,title);
notebook_name_raw = strcat(notebook_name{1}, '_raw', '.xlsx');

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
    vector_ca = [(rb4(1) - rb5(1))/1000, (rb4(2) - rb5(2))/1000];

    theta = atan(vector_ca(2)/vector_ca(1));
    
    % Find xR, yR, thetaR - Real values

    xR = rb5(1);
    yR = rb5(2);
    thetaR = theta;
 
    %determine starting position of the robot
    if (xR~=0 && yR~=0 && firstLine == 0 )
        xRi = xR;
        yRi = yR;
        firstLine=1;
    
    elseif (firstLine~=0)
        % Find xD, yD, thetaD - Desired values

        [xD, yD, thetaD] = expectedPath(timeglobal);

        xD = xD+xRi;
        yD = yD+yRi;
        thetaD = 0;

        % Error terms

        xE = xD - xR;
        yE = yD - yR;
        thetaE = thetaD - thetaR;

        thetaE = min(thetaE, pi-thetaE);

        kp = [0.05,0.05,0.2;+0.05,-0.05,0.2;-0.05,-0.05,0.2;-0.05,0.05,0.2].*5;
        ki = [0.03,0.03,0.01;0.03,-0.03,0.01;-0.03,-0.03,0.01;-0.03,+0.03,0.01].*1;

        err = [xE; yE; thetaE]
        integrator_error = integrator_error + err;

        u = kp*err + ki*err;
        %u = kp*err

        t=timeglobal;

        if (abs(u(1)) > 160 || abs(u(2)) > 160 || abs(u(3)) > 160 || abs(u(4)) > 160)
            u1 = u(1);
            u2 = u(2);
            u3 = u(3);
            u4 = u(4);
            big = max(u1, u2);
            big = max(big, u3);
            big = max(big, u4);
            u = u./abs(big)*160;
        end

        [u', err', integrator_error'];
        volts_to_send = sprintf('%d,%d,%d,%d*', int16(u(1)), int16(u(2)), int16(u(3)), int16(u(4)))
        fprintf(serialPortObj, volts_to_send);

        Sheet1Mat(matcounter,:) = [timeglobal, xR, yR, thetaR, xD, yD, thetaD, xE, yE, thetaE, u(1), u(2), u(3), u(4)]; % Gives raw data

        matcounter = matcounter + 1;

        timeglobal = toc(tglobal);
    end
end % end of while loop, everything before this runs until the end of the script

fprintf(serialPortObj, '0,0,0,0*');

xlswrite("C:/Users/Vicon/Desktop/Git/Omniwheel VICON Code/Raw Data/" + notebook_name_raw, Sheet1Mat);