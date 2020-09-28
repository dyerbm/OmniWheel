clc
clear all

%% open serial port
delete(instrfind);
port = 'COM17'; % Replace with whatever the USB serial bus from the XBee module is on (was 7)
serialPortObj = serial(port, 'BaudRate', 9600);
fopen(serialPortObj);

%% Read in trajectory
% Change the path as necissary
filename = 'C:\Users\jdiro\Desktop\Git\PathPlanning\Search_based_Planning\Search_2D\testpath.csv';
delimiter = ',';
startRow = 2;
formatSpec = '%f%f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(fileID);
path = table(dataArray{1:end-1}, 'VariableNames', {'x','y','theta'});
path = path{:,:};
path = path/100.; %convert from cm to m

%% Create output file
output_filename = datestr(now,'yyyy-mm-dd HH-MM')+"xlsx";
output_table = zeros(length(path),4);

%% Send data to robot
timerval = tic;
for i=1:length(path)
    tic
    
    fprintf(serialPortObj, strcat(num2str(path(i,1)),",",num2str(path(i,2)),",",num2str(path(i,3)),"*"));
    output_table(i,:)=[now , path(i,1) , path(i,2), path(i,3)];
    
    strcat(num2str(path(i,1)),",",num2str(path(i,2)),",",num2str(path(i,3)),"*");
    
    pause(0.02-toc) %send the data at the right speed
end

xlswrite(strcat('C:\Users\jdiro\Desktop\Git\Sensing\Data\', output_filename),output_table)