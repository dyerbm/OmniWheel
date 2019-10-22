function [xD, yD, thetaD] = expectedPath(time)
    
    %% move in a line
%     %xD = 0;
%     %yD = time*400;
%     yD = 0;
%     xD = -time*200;
%     thetaD = 0;

    %% square
    
    
    %% Do a circle
%     radius=800;
%     circleTime = 10;
%     yD=radius*sin(2*pi/circleTime*time);
%     xD=radius*cos(2*pi/circleTime*time)-radius;
%     thetaD=atan(1)-pi/4;


    %% Rose Curve Path
    circleTime=10;
    n=2; %must be an even number (2*n=number of pedals)
    a=300; %radius of rose curve
    yD=a*cos(n*2*pi/circleTime*time)*sin(2*pi/circleTime*time);
    xD=a*cos(n*2*pi/circleTime*time)*cos(2*pi/circleTime*time);
    thetaD=atan(1)-pi/4;
    
end