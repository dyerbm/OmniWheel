function [vels, xFrictions, yFrictions, tFrictions]=FFFunction(data, start, interval, maxVel, binSize)

    numBins = int16(maxVel*2/binSize)+1;
    vels=linspace(-1*maxVel,maxVel,numBins);

    xInput={};
    xOutput={};
    yInput={};
    yOutput={};
    tInput={};
    tOutput={};

    xInput{numBins+1}=0; %initialize a cell just out of the range
    xOutput{numBins+1}=0;
    yInput{numBins+1}=0;
    yOutput{numBins+1}=0;
    tInput{numBins+1}=0;
    tOutput{numBins+1}=0;

    xFrictions = zeros(numBins,1);
    yFrictions = zeros(numBins,1);
    tFrictions = zeros(numBins,1);

    for i=start:interval:size(data,1)-1

        %calculate current position
        vector_ca1 = [(data.rb4x(i) - data.rb5x(i))/1000, (data.rb4y(i) - data.rb5y(i))/1000];
        theta1 = atan(vector_ca1(2)/vector_ca1(1));
        %calculate next position
        vector_ca2 = [(data.rb4x(i+1) - data.rb5x(i+1))/1000, (data.rb4y(i+1) - data.rb5y(i+1))/1000];
        theta2 = atan(vector_ca2(2)/vector_ca2(1));

        %calculate velocity FIX TIMESTEP
        robotvel = [(data.rb5x(i+1)-data.rb5x(i))/1000,(data.rb5y(i+1)-data.rb5y(i))/1000]/(data.globalTime(i+1)-data.globalTime(i));
        %robotvel = [(data.rb5x(i+1)-data.rb5x(i))/1000,(data.rb5y(i+1)-data.rb5y(i))/1000]/0.02;
        RotationMatrix = [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)];
        robotXOrientation = [cos(pi/4) sin(pi/4)]*RotationMatrix;
        robotYOrientation = [cos(3*pi/4) sin(3*pi/4)]*RotationMatrix;
        robotxvel = dot(robotvel,robotXOrientation)/norm(robotXOrientation);
        robotyvel = dot(robotvel,robotYOrientation)/norm(robotYOrientation);
        robottvel = (theta2-theta1)/(data.globalTime(i+1)-data.globalTime(i));
        
        dvector = [(data.rb5x(i+1)-data.rb5x(i))/1000,(data.rb5y(i+1)-data.rb5y(i))/1000];
        x_diff = dot(dvector,robotXOrientation)/norm(robotXOrientation);
        y_diff = dot(dvector,robotYOrientation)/norm(robotYOrientation);
        
        %set up matrices
        pwm=[-data.u2(i)+data.u4(i),0,0;0,data.u1(i)-data.u3(i),0;0,0,data.u1(i)+data.u2(i)+data.u3(i)+data.u4(i)];
        %xk=[robotxvel,robotyvel,robottvel];
        xk=[data.rb5x(i)/1000,data.rb5y(i)/1000,theta1];
        %Ei=[diag(xk),pwm,eye(3)];
        %Ei = diag(xk)*[0.96788;0.9661;0.6693]+pwm*[8.1846e-6;-10.803e-6;3.3585e-4];%based on velocity
        Ei = diag(xk)*[1;1;1]+pwm*[2.0e-6;2.7e-6;3.99e-6];%based on position
        %Ei = diag(xk)*[1;1;1]+pwm*[4.3e-5;4.3e-5;3.99e-6];%based on position

        %check which x and y velocity bins the values should be in
        xbin=int16(round(robotxvel/binSize))+(numBins+1)/2;
        ybin=int16(round(robotyvel/binSize))+(numBins+1)/2;
        tbin=int16(round(robottvel/binSize))+(numBins+1)/2;

        %append matrices to input and outputs
        if abs(data.globalTime(i+1)-data.globalTime(i)-0.02)<0.0001
            if 0<xbin && xbin<=numBins
                xInput{xbin}=[xInput{xbin}; Ei];
                %xOutput{xbin}=[xOutput{xbin};data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
                xOutput{xbin}=[xOutput{xbin};data.rb5x(i)/1000+x_diff;data.rb5y(i)/1000+y_diff;theta2];
            end
            if 0<ybin && ybin<=numBins
                yInput{ybin}=[yInput{ybin};Ei];
                %yOutput{ybin}=[yOutput{ybin};data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
                yOutput{ybin}=[yOutput{ybin};data.rb5x(i)/1000+x_diff;data.rb5y(i)/1000+y_diff;theta2];
            end
            if (0<tbin && tbin<=numBins)
                tInput{tbin}=[tInput{tbin};Ei];
                %tOutput{tbin}=[tOutput{tbin};data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
                tOutput{tbin}=[tOutput{tbin};data.rb5x(i)/1000+x_diff;data.rb5y(i)/1000+y_diff;theta2];
            end
        end
    end

    format long
    xOutput

    for i=1:numBins
        if length(xInput{i})~= 0
            I=eye(3);
            for j=2:(size(xInput{i},1)/3)
                I=[I;eye(3)];
            end
            P=I\(xOutput{i}-xInput{i});
            xFrictions(i)=P(1);
        end
        if length(yInput{i})~= 0
            I=eye(3);
            for j=2:(size(yInput{i},1)/3)
                I=[I;eye(3)];
            end
            P=I\(yOutput{i}-yInput{i});
            yFrictions(i)=P(2);
        end
        if length(tInput{i})~= 0
            I=eye(3);
            for j=2:(size(tInput{i},1)/3)
                I=[I;eye(3)];
            end
            P=I\(tOutput{i}-tInput{i});
            tFrictions(i)=P(3);
        end
    end






end