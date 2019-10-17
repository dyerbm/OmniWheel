timenow=0.01;
timeglobal=0;
tglobal=tic;

steady = 0;

rb1=[0,0,0];
rb2=[0,0,0];
rb3=[0,0,0];
rb4=[0,0,0];
rb5=[0,0,0];

counter = 0;

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

volts_to_send = "0,0,0,0*";

fprintf(serialPortObj, volts_to_send);
tglobal=tic;


%% GET DATA

while(steady == 0)
    
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

    if counter > 10
    %% Processing the data in real time

        % Getting angle relative to x axis.
        vector_ca = [(rb4(1) - rb5(1))/1000, (rb4(2) - rb5(2))/1000];

        theta = atan(vector_ca(2)/vector_ca(1));

        % Find xR, yR, thetaR - Real values

        xR = rb5(1);
        yR = rb5(2);
        thetaR = theta;

        % Find xD, yD, thetaD - Desired values

        xD = 500;
        yD = 500;
        thetaD = 0;

        % Error terms

        xE = xD - xR
        yE = yD - yR
        thetaE = thetaD - thetaR

        thetaE = min(thetaE, pi-thetaE);

        if (abs(thetaE)-thetaD > 0.2)
            if thetaE > 0
                u = [0,0,6,0];
            else
                u = [0,-0,-6,-0];
            end
            t=1
        elseif (abs(xE) > xD * 0.1)
            if xE > 0
                u = [3,3,-3,-3];
            else
                u = [-3,-3,3,3];
            end
            t=2
        elseif (abs(yE) > yD * 0.1)
            if yE > 0
                u = [3,-3,-3,3];
            else
                u = [-3,3,3,-3];
            end
            t=3
        else
            steady = 1;
        end    

        volts_to_send = sprintf('%d.5,%d.5,%d.5,%d.5*', u(1), u(2), u(3), u(4));
        fprintf(serialPortObj, volts_to_send);
        %steady = 1

    end
    counter = counter + 1;
end % end of while loop, everything before this runs until the end of the script

fprintf(serialPortObj, '0,0,0,0*');