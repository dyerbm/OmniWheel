timenow=0.01;
timeglobal=0;
tglobal=tic;
u=[0,0,0,0];

marker_1_lost = false;
marker_2_lost = false;
marker_3_lost = false;
marker_4_lost = false;
marker_5_lost = false;

matcounter = 1; % Starting row for output matrix
max_operation = 3; % Maximum time robot will move
matrixsize = max_operation * 100 + 100; % Based on the time for operation, will wait 1 second after robot stops to end recording
 
Sheet1Mat = zeros(matrixsize,21);

% Below are the headings for the files commented, feel 
%Raw_Headings = ['GlobalTime', 'Time', 'M1X', 'M1Y', 'M1Z', 'M2X', 'M2Y', 'M2Z', 'M3X', 'M3Y', 'M3Z', 'M4X', 'M4Y', 'M4Z', 'M5X', 'M5Y', 'M5Z', 'Voltages'];

%% select serial port.
delete(instrfind);
port = 'COM7'; % Replace with whatever the USB serial bus from the XBee module is on (was 7)
serialPortObj = serial(port, 'BaudRate', 9600);
fopen(serialPortObj);

%% get data for the operating volatage range for the robot
prompt={'Enter voltages to be sent to motors (range is -12 to 12, separate by ,) eg. 0,0,0,0'};
title='Voltage Selection';
answer = inputdlg(prompt,title);
volts_to_send = strcat(answer{1}, '*');
u = (sscanf(volts_to_send,'%d,%d,%d,%d*'))';

%% Get name of notebook from user
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

fprintf(serialPortObj, volts_to_send);
tglobal=tic;

rb1 = [0,0,0];
rb2 = [0,0,0];
rb3 = [0,0,0];
rb4 = [0,0,0];
rb5 = [0,0,0];

%% GET DATA

while(matcounter <= matrixsize)
    
    % 1 - Get a frame
    while MyClient.GetFrame().Result.Value ~= Result.Success
    end
    
    % Print the frame number
    Output_GetFrameNumber = MyClient.GetFrameNumber();
    
    marker_1_lost = false;
    marker_2_lost = false;
    marker_3_lost = false;
    marker_4_lost = false;
    marker_5_lost = false;
    
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
                            marker_5_lost = false;
                        else
                            if matcounter > 3
                                marker_5_lost = true;
                            end
                        end
                        
                    end
                    if strcmpi(MarkerName, 'rb1')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb1 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                            marker_1_lost = false;
                        else
                            if matcounter > 3
                                marker_1_lost = true;
                            end
                        end
                        
                    end
                    if strcmpi(MarkerName, 'rb2')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb2 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                            marker_2_lost = false;
                        else
                            if matcounter > 3
                                marker_2_lost = true;
                            end
                        end
                        
                    end
                    if strcmpi(MarkerName, 'rb3')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb3 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                            marker_3_lost = false;
                        else
                            if matcounter > 3
                                marker_3_lost = true;
                            end
                        end
                        
                    end
                    if strcmpi(MarkerName, 'rb4')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb4 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                            marker_4_lost = false;
                        else
                            if matcounter > 3
                                marker_4_lost = true;
                            end
                        end
                        
                    end
                    
                end % end of if for checking to make sure the marker is valid
                
            end % end of for loop for checking markers
            
        end % end of robot segment
        

        % Save Sheet1 Data
        Sheet1Mat(matcounter,:) = [timeglobal, timenow, rb1, rb2, rb3, rb4, rb5, u]; % Gives raw data
        
        matcounter = matcounter + 1;
        
        % At 100HZ, this value should be fixed at 10ms between each
        % iteration
        timenow = 0.010;
        timeglobal = toc(tglobal);

    end

end % end of while loop, everything before this runs until the end of the script

volts_to_send = '0,0,0,0*';
u = [0,0,0,0];
fprintf(serialPortObj, volts_to_send);
xlswrite("C:\Users\Vicon\Desktop\Biglar Begian VICON\" + notebook_name_raw, Sheet1Mat);