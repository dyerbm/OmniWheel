function [xD, yD, thetaD] = expectedPath(time)
    
    %move in a line
    xD = 0;
    %yD=0;
    yD = time*200;
    %xD = time*200;
    thetaD = atan(1)-pi/4;

    
    
%     %Do a circle
%     radius=100;
%     circleTime = 5;
%     yD=radius*sin(2*pi/circleTime*time);
%     xD=radius*cos(2*pi/circleTime*time);
%     thetaD=atan(1)-pi/4;s


    %Rose Curve Path
    circleTime=5;
    n=4; %must be an even number (2*n=number of pedals)
    a=100; %radius of rose curve
    yD=a*cos(n*2*pi/circleTime*time)*sin(2*pi/circleTime*time);
    xD=a*cos(n*2*pi/circleTime*time)*cos(2*pi/circleTime*time);
    thetaD=atan(1)-pi/4;
    
end