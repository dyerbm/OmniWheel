clear all
clc
close all

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
binSize=0.01;
numBins = int16(maxVel*2/binSize)+1;

%calculate frictions based on 3 different data sets
[vels1,xFrictions1,yFrictions1,tFrictions1]=FFFunction(data,1,3,maxVel,binSize);
[vels2,xFrictions2,yFrictions2,tFrictions2]=FFFunction(data,2,3,maxVel,binSize);
[vels3,xFrictions3,yFrictions3,tFrictions3]=FFFunction(data,3,3,maxVel,binSize);

%calculate frictions for curve fitting
[vels4,xFrictions4,yFrictions4,tFrictions4]=FFFunction(data,1,1,maxVel,binSize);


%% Create Figures
%repeatability of y frictions
figure;
scatter(vels1(1:numBins),yFrictions1(1:numBins)); hold on;
scatter(vels2(1:numBins),yFrictions2(1:numBins)); hold on;
scatter(vels3(1:numBins),yFrictions3(1:numBins)); hold off;
ylabel("f_2^d");xlabel("normal velocity (m/s)");
legend("dataset 1", "dataset 2", "dataset 3");

%repeatability of x friction
figure;
scatter(vels1(1:numBins),xFrictions1(1:numBins)); hold on;
scatter(vels2(1:numBins),xFrictions2(1:numBins)); hold on;
scatter(vels3(1:numBins),xFrictions3(1:numBins)); hold off;
ylabel("f_1^d");xlabel("velocity (m/s)");
legend("dataset 1", "dataset 2", "dataset 3");

%repeatability of theta friction
figure;
scatter(vels1(1:numBins),tFrictions1(1:numBins)); hold on;
scatter(vels2(1:numBins),tFrictions2(1:numBins)); hold on;
scatter(vels3(1:numBins),tFrictions3(1:numBins)); hold off;
ylabel("f_3^d");xlabel("velocity (rad/s)");
legend("dataset 1", "dataset 2", "dataset 3");


%curve fitting for x friction
figure;
scatter(vels4(1:numBins),xFrictions4(1:numBins)); hold on;
ft = fittype( 'poly5' );
opts = fitoptions( 'Method', 'LinearLeastSquares' );
opts.Normalize = 'on';
opts.Robust = 'Bisquare';
[fitresult, gof] = fit( vels4(1:numBins)',xFrictions4(1:numBins), ft );
h = plot(fitresult, vels4(1:numBins)',xFrictions4(1:numBins));
ylabel("f_1^d");xlabel("velocity (m/s)");
%ylim([-0.05,0.05]);
%legend("Full dataset",h);


%curve fitting for y friction
figure;
scatter(vels4(1:numBins),yFrictions4(1:numBins)); hold on;
ft = fittype( 'poly7' );
opts = fitoptions( 'Method', 'LinearLeastSquares' );
opts.Normalize = 'on';
opts.Robust = 'Bisquare';
[fitresult, gof] = fit( vels4(1:numBins)',yFrictions4(1:numBins), ft );
h = plot(fitresult, vels4(1:numBins)',yFrictions4(1:numBins));
ylabel("f_2^d");xlabel("velocity (m/s)");
%ylim([-0.05,0.05]);
%legend("Full dataset",h);


%curve fitting for theta friction
figure;
scatter(vels4(1:numBins),tFrictions4(1:numBins)); hold on;
ft = fittype( 'poly1' );
opts = fitoptions( 'Method', 'LinearLeastSquares' );
opts.Normalize = 'on';
opts.Robust = 'Bisquare';
[fitresult, gof] = fit( vels4(1:numBins)',tFrictions4(1:numBins), ft );
h = plot(fitresult, vels4(1:numBins)',tFrictions4(1:numBins));
ylabel("f_3^d");xlabel("velocity (m/s)");
%ylim([-0.05,0.05]);
%legend("Full dataset",h);




