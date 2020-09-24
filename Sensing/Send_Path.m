clc
clear all

%% open serial port
delete(instrfind);
port = 'COM17'; % Replace with whatever the USB serial bus from the XBee module is on (was 7)
serialPortObj = serial(port, 'BaudRate', 9600);
fopen(serialPortObj);

%% Read in trajectory
% Change the path as necissary
filename = 'C:\Users\jdiro\Desktop\Git\PathPlanning\Search_based_Planning\Search_2D\Path\testoutput.csv';
delimiter = ',';
startRow = 2;
formatSpec = '%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(fileID);
path = table(dataArray{1:end-1}, 'VariableNames', {'x','y','theta'});
path = path{:,:};
path = path/100.; %convert from cm to m

%% Send data to robot
timerval = tic;
for i=1:size(path)
    tic
    
    fprintf(serialPortObj, strcat(num2str(path(i,1)),",",num2str(path(i,2)),",",num2str(path(i,3)),"*"));
    
    value=toc(timerval)
    
    pause(0.02-toc) %send the data at the right speed
end