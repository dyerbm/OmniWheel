function [yd, dyd, ddyd] = desired_trajectory(t)
    
    %% move in a line
    yd=[1000;1000;0];
    dyd=[0;0;0];
    ddyd=[0;0;0];

    %% square
    
    
    %% Do a circle
%     r=1*1000;
%     circtime=10;
%     yd = [r*cos(2*pi/circtime*t);r*sin(2*pi/circtime*t);0];
%     dyd = [-r*2*pi/circtime*sin(2*pi/circtime*t);r*2*pi/circtime*cos(2*pi/circtime*t);0];
%     ddyd = [-r*2*pi/circtime*2*pi/circtime*cos(2*pi/circtime*t);-r*2*pi/circtime*2*pi/circtime*sin(2*pi/circtime*t);0];


    %% Rose Curve Path
%     circleTime=10;
%     n=2; %must be an even number (2*n=number of pedals)
%     a=300; %radius of rose curve
%     yD=a*cos(n*2*pi/circleTime*time)*sin(2*pi/circleTime*time);
%     xD=a*cos(n*2*pi/circleTime*time)*cos(2*pi/circleTime*time);
%     thetaD=atan(1)-pi/4;
    
end