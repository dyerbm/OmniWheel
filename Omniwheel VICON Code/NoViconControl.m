clc

timenow=0.01;
timeglobal=0;
tglobal=tic;

matcounter = 1; % Starting row for output matrix
max_operation = 16; % Maximum time robot will move
matrixsize = max_operation * 100 + 100; % Based on the time for operation, will wait 1 second after robot stops to end recording
 
Sheet1Mat = zeros(matrixsize,14);

circleCount=0;
v1=0;
v2=0;
v3=0;
v4=0;

rb1=[0,0,0];
rb2=[0,0,0];
rb3=[0,0,0];
rb4=[0,0,0];
rb5=[0,0,0];


delete(instrfind);
port = 'COM17'; % Replace with whatever the USB serial bus from the XBee module is on (was 7)
serialPortObj = serial(port, 'BaudRate', 9600);
fopen(serialPortObj);

fprintf(serialPortObj,'0,0,0,0*');


while(timeglobal <= max_operation)
    %volts_to_send
    %%%%%%%%%%%Move in a line%%%%%%%%%%%%%%%
%     volts_to_send = "100,100,100,100*";
%     %volts_to_send = "150,160,170,180*";
%     volts_to_send = "156,167,178,189*";
%     %volts_to_send = "255,175,186,197*";
%     fprintf(serialPortObj, volts_to_send)
%     pause(0.1)
    
    
    
    circleTime=8;
    speed=250;
    v1=int16(sin(2*pi/circleTime*timeglobal)*speed);
    v3=int16(-sin(2*pi/circleTime*timeglobal)*speed);
    v2=int16(cos(2*pi/circleTime*timeglobal)*speed);
    v4=int16(-cos(2*pi/circleTime*timeglobal)*speed);
    
    volts_to_send=strcat(int2str(v1),',',int2str(v2),',',int2str(v3),',',int2str(v4),'*')
    
    fprintf(serialPortObj, volts_to_send);
    pause(0.05);
    timeglobal=toc(tglobal)
    fscanf(serialPortObj)
end
    

volts_to_send='0,0,0,0*';
fprintf(serialPortObj,volts_to_send);
pause(0.2);
fprintf(serialPortObj,volts_to_send);
    
    
    
    
    