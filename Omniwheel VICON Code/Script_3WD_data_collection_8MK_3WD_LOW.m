% A lot of this code was provided by Amit Patel
% It has been modified to work with a different robot at a different
% frequency for custom trajectories


timenow=0.02;
timeglobal=0;
tglobal=tic;
desired=[0,0,0];

marker_1_lost = false;
marker_2_lost = false;
marker_3_lost = false;
marker_4_lost = false;
marker_5_lost = false;
marker_6_lost = false;
marker_7_lost = false;
marker_8_lost = false;

matcounter = 1; % Starting row for output matrix
max_operation = 120; % Time to record (max time robot will move if sending signal)
matrixsize = max_operation * 50 + 50; % Based on the time for operation, will wait 1 second after robot stops to end recording
 
Sheet1Mat = zeros(matrixsize,29);

% Below are the headings for the files commented, feel 
%Raw_Headings = ['GlobalTime', 'Time', 'M1X', 'M1Y', 'M1Z', 'M2X', 'M2Y', 'M2Z', 'M3X', 'M3Y', 'M3Z', 'M4X', 'M4Y', 'M4Z', 'M5X', 'M5Y', 'M5Z','M6X', 'M6Y', 'M6Z','M7X', 'M7Y', 'M7Z','M8X', 'M8Y', 'M8Z'];

%% select serial port.
delete(instrfind);
port = 'COM7'; % Replace with whatever the USB serial bus from the XBee module is on (was 7)
serialPortObj = serial(port, 'BaudRate', 9600);
fopen(serialPortObj);

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

rb1 = [0,0,0];
rb2 = [0,0,0];
rb3 = [0,0,0];
rb4 = [0,0,0];
rb5 = [0,0,0];
rb6 = [0,0,0];
rb7 = [0,0,0];
rb8 = [0,0,0];

%% GET DATA
tglobal=tic;
while(matcounter <= matrixsize)
    
    % 1 - Get a frame
    while MyClient.GetFrame().Result.Value ~= Result.Success
    end
    timeglobal = toc(tglobal);
    
    % Print the frame number
    Output_GetFrameNumber = MyClient.GetFrameNumber();
    
    marker_1_lost = false;
    marker_2_lost = false;
    marker_3_lost = false;
    marker_4_lost = false;
    marker_5_lost = false;
    marker_6_lost = false;
    marker_7_lost = false;
    marker_8_lost = false;
    
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
        if strcmpi(SubjectName, '8MK_3WD_LOW')
            % Get translation data of all 4 markers
            MarkerCount = MyClient.GetMarkerCount( SubjectName ).MarkerCount;
%             fprintf( '    Markers (%d):\n', MarkerCount );
            for MarkerIndex = 1:MarkerCount
                MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;
                
                if MarkerName ~= ' ' % error check - it is labelled

                    MarkerTranslation = MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );
                    
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
                    if strcmpi(MarkerName, 'rb6')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb6 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                            marker_6_lost = false;
                        else
                            if matcounter > 3
                                marker_6_lost = true;
                            end
                        end
                        
                    end
                    if strcmpi(MarkerName, 'rb7')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb7 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                            marker_7_lost = false;
                        else
                            if matcounter > 3
                                marker_7_lost = true;
                            end
                        end
                        
                    end
                    if strcmpi(MarkerName, 'rb8')
                        
                        %Added for filling missed frames
                        if MarkerTranslation.Translation(1) ~= 0 && MarkerTranslation.Translation(2) ~= 0 && MarkerTranslation.Translation(3) ~= 0
                            rb8 = [MarkerTranslation.Translation(1) MarkerTranslation.Translation(2) MarkerTranslation.Translation(3)];
                            marker_8_lost = false;
                        else
                            if matcounter > 3
                                marker_8_lost = true;
                            end
                        end
                        
                    end
                    
                end % end of if for checking to make sure the marker is valid
                
            end % end of for loop for checking markers
            
        end % end of robot segment
        
        % At 100HZ, this value should be fixed at 10ms between each
        % iteration
        timenow = 0.020;
        
        %%%-------------set desired position---------------------%%%
        %fprintf(serialPortObj, '1,0,0*');
        
        Period=60; %period in seconds
        rose = 2; %determines number of pedals (2k pedals for even k, k pedals for odd k)
        
        %desired = [1,0,0];
        %desired = [sin(2*pi*timeglobal/Period),cos(2*pi*timeglobal/Period), 0]; %Circle (r=1m)
        %desired = [cos(rose*2*pi*timeglobal/Period)*cos(2*pi*timeglobal/Period), cos(rose*2*pi*timeglobal/Period)*sin(2*pi*timeglobal/Period),0];
        
        %desired = [0 0 2*pi*timeglobal/12.]; %pspin
        %desired = [0 0 -2*pi*timeglobal/12.]; %nspin
        desired = [0 0 sin(2*pi*timeglobal/24.)*0.9*pi]; %sin
        
        fprintf(serialPortObj, strcat(num2str(desired(1)),",",num2str(desired(2)),",",num2str(desired(3)),"*"));
        desired
        
        % Save Sheet1 Data
        format long
        Sheet1Mat(matcounter,:) = [timeglobal, timenow, rb1, rb2, rb3, rb4, rb5, rb6, rb7, rb8, desired]; % Gives raw data

        matcounter = matcounter + 1;

    end

end % end of while loop, everything before this runs until the end of the script


desired = [0,0,0];
fprintf(serialPortObj, '0,0,0*');
%xlswrite("./Raw Data/test_r.xlsx", Sheet1Mat);
xlswrite(notebook_name_raw, Sheet1Mat);