clc
clear all

%open serial port
delete(instrfind);
port = 'COM17'; % Replace with whatever the USB serial bus from the XBee module is on (was 7)
serialPortObj = serial(port, 'BaudRate', 9600);
fopen(serialPortObj);

