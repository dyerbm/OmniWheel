function [yd, dyd, ddyd] = desired_trajectory(t)
    
    %% move in a line
    yd=[-500;0;0];
    dyd=[0;0;0];
    ddyd=[0;0;0];

    %% square
    
    
    %% Do a circle
%     r=0.4;
%     yd = [r*cos(t);r*sin(t);0];
%     dyd = [-r*sin(t);r*cos(t);0];
%     ddyd = [-r*cos(t);-r*sin(t);0];


    %% Rose Curve Path
%     circleTime=10;
%     n=2; %must be an even number (2*n=number of pedals)
%     a=300; %radius of rose curve
%     yD=a*cos(n*2*pi/circleTime*time)*sin(2*pi/circleTime*time);
%     xD=a*cos(n*2*pi/circleTime*time)*cos(2*pi/circleTime*time);
%     thetaD=atan(1)-pi/4;
    
end