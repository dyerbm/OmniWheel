
results = []
Velocity = []

data=[];
%rawDataPath = "C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\spinData\";
%data=getData(data,rawDataPath+int2str(i)+"spin_raw.xlsx","A1400:U4000");
%data=getData(data,rawDataPath+"spinNeg150_raw.xlsx","A500:U1000");
data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"2019-12-10_circ10s_neg160pwm_raw.xlsx","A100:U1700");
data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"2019-12-10_circ10s_160pwm_raw.xlsx","A100:U1700");
%data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"250circ2_raw.xlsx","A1400:U3700");

%Find Friction
maxVel = 0.5;
binSize=0.05;
numBins = (maxVel*2/binSize)+1;
vels=linspace(-1*maxVel,maxVel,numBins);

xInput={};
xOutput={};
yInput={};
yOutput={};

xInput{numBins+1}=0; %initialize a cell just out of the range
xOutput{numBins+1}=0;
yInput{numBins+1}=0;
yOutput{numBins+1}=0;

xFrictions = zeros(numBins,1);
yFrictions = zeros(numBins,1);

for i=20:2:size(data,1)-100
    
    %calculate current position
    vector_ca1 = [(data.rb4x(i) - data.rb5x(i))/1000, (data.rb4y(i) - data.rb5y(i))/1000];
    theta1 = atan(vector_ca1(2)/vector_ca1(1));
    %calculate next position
    vector_ca2 = [(data.rb4x(i+1) - data.rb5x(i+1))/1000, (data.rb4y(i+1) - data.rb5y(i+1))/1000];
    theta2 = atan(vector_ca2(2)/vector_ca2(1));
    %set up matrices
    pwm=[data.u2(i)-data.u4(i),0,0;0,data.u1(i)-data.u3(i),0;0,0,data.u1(i)+data.u2(i)+data.u3(i)+data.u4(i)];
    xk=[data.rb5x(i)/1000,data.rb5y(i)/1000,theta1];
    Ei=[diag(xk),pwm,eye(3)];
    
    %calculate velocity FIX TIMESTEP
    %robotvel = [(data.rb5x(i+1)-data.rb5x(i))/1000,(data.rb5y(i+1)-data.rb5y(i))/1000]/(data.globalTime(i+1)-data.globalTime(i));
    robotvel = [(data.rb5x(i+1)-data.rb5x(i))/1000,(data.rb5y(i+1)-data.rb5y(i))/1000]/0.02;
    RotationMatrix = [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)];
    robotXOrientation = [cos(pi/4) sin(pi/4)]*RotationMatrix;
    robotYOrientation = [cos(3*pi/4) sin(3*pi/4)]*RotationMatrix;
    robotxvel = dot(robotvel,robotXOrientation)/norm(robotXOrientation);
    robotyvel = dot(robotvel,robotYOrientation)/norm(robotYOrientation);
    
    %check which x and y velocity bins the values should be in
    xbin=int16(round(robotxvel/binSize))+(numBins+1)/2;
    ybin=int16(round(robotyvel/binSize))+(numBins+1)/2;
    
    %append matrices to input and outputs
    
    if 0<xbin && xbin<numBins
        xInput{xbin}=[xInput{xbin}; Ei];
        xOutput{xbin}=[xOutput{xbin};data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
    end
    if 0<ybin && ybin<numBins
        yInput{ybin}=[yInput{ybin};Ei];
        yOutput{ybin}=[yOutput{ybin};data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
    end
end

format long

for i=1:numBins
    if length(xInput{i})~= 0
        P=lsqminnorm(xInput{i},xOutput{i},'warn');
        xFrictions(i)=P(7);
    end
    if length(yInput{i})~= 0
        P=lsqminnorm(yInput{i},yOutput{i},'warn');
        yFrictions(i)=P(8);
    end
end

%results=[results; P(9)];
%Velocity = [Velocity; mean(vels)];

results
Velocity


