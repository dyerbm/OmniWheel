clear all
clc
%close all

results = []
Velocity = []

data=[];
rawDataPath = "C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\50Hz\";
%data=getData(data,rawDataPath+int2str(i)+"spin_raw.xlsx","A1400:U4000");
%data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"spinNeg150_raw.xlsx","A500:U1000");
%data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"spin150_raw.xlsx","A500:U1000");
%data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"2019-12-10_circ10s_neg160pwm_raw.xlsx","A500:U2200");
%data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"2019-12-10_circ10s_160pwm_raw.xlsx","A500:U1000");
%data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"250circ2_raw.xlsx","A200:U3700");
%data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\circspinData\"+"90spin160circle10seconds.xlsx","A100:U400");
% 
% data=getData(data,rawDataPath+"200negCirc.xlsx","A50:U2500");
% data=getData(data,rawDataPath+"200Circ.xlsx","A50:U2500");
% data=getData(data,rawDataPath+"200varspin.xlsx","A50:U5000");

data=getData2(data,rawDataPath+"200negCirc.xlsx",50,2500);
data=getData2(data,rawDataPath+"200Circ.xlsx",50,2500);
data=getData2(data,rawDataPath+"200varspin.xlsx",50,5000);

%Find Friction
maxVel = 0.7;
binSize=0.02;

[vels,xFrictions,yFrictions,tFrictions]=FFFunction(data,1,2,maxVel,binSize);
numBins = int16(maxVel*2/binSize)+1;



figure;scatter(vels(1:numBins),yFrictions(1:numBins));ylabel("f_2^d");xlabel("normal velocity (m/s)")


figure;scatter(vels(1:numBins),xFrictions(1:numBins));ylabel("f_1^d");xlabel("velocity (m/s)")

%figure;scatter((vels(1:numBins)),log(xFrictions(1:numBins)))



