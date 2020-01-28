%% Setup the Import Options

% opts = spreadsheetImportOptions("NumVariables", 21);
% 
% % Specify sheet and range
% opts.Sheet = "Sheet1";
% opts.DataRange = "A500:U1000";
% 
% % Specify column names and types
% opts.VariableNames = ["globalTime", "timeNow", "rb1x", "rb1y", "rb1z", "rb2x", "rb2y", "rb2z", "rb3x", "rb3y", "rb3z", "rb4x", "rb4y", "rb4z", "rb5x", "rb5y", "rb5z", "u1", "u2", "u3", "u4"];
% opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
% 
% % Import the data
% data = readtable("C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\2019-12-10_circ10s_160pwm_raw.xlsx", opts, "UseExcel", false);
% 
% 
% %% Clear temporary variables
% clear opts
% 
% %% Setup the Import Options
% opts = spreadsheetImportOptions("NumVariables", 21);
% 
% % Specify sheet and range
% opts.Sheet = "Sheet1";
% opts.DataRange = "A500:U1000";
% 
% % Specify column names and types
% opts.VariableNames = ["globalTime", "timeNow", "rb1x", "rb1y", "rb1z", "rb2x", "rb2y", "rb2z", "rb3x", "rb3y", "rb3z", "rb4x", "rb4y", "rb4z", "rb5x", "rb5y", "rb5z", "u1", "u2", "u3", "u4"];
% opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
% 
% % Import the data
% data = [data; readtable("C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\2019-12-10_circ10s_neg160pwm_raw.xlsx", opts, "UseExcel", false)];
% %data = [readtable("C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\2019-12-10_circ10s_neg160pwm_raw.xlsx", opts, "UseExcel", false)];
% 
% 
% %% Clear temporary variables
% clear opts
% 
% 
% %% Setup the Import Options
% opts = spreadsheetImportOptions("NumVariables", 21);
% 
% % Specify sheet and range
% opts.Sheet = "Sheet1";
% opts.DataRange = "A500:U1000";
% 
% % Specify column names and types
% opts.VariableNames = ["globalTime", "timeNow", "rb1x", "rb1y", "rb1z", "rb2x", "rb2y", "rb2z", "rb3x", "rb3y", "rb3z", "rb4x", "rb4y", "rb4z", "rb5x", "rb5y", "rb5z", "u1", "u2", "u3", "u4"];
% opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
% 
% % Import the data
% data = [data; readtable("C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\spin150_raw.xlsx", opts, "UseExcel", false)];
% 
% 
% %% Clear temporary variables

%% Read in data
data=[];
rawDataPath = "C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\50Hz\";
%data=getData(data,rawDataPath+"spin150_raw.xlsx","A500:U1000");
%data=getData(data,rawDataPath+"spinNeg150_raw.xlsx","A500:U1000");
%data=getData(data,rawDataPath+"2019-12-10_circ10s_neg160pwm_raw.xlsx","A100:U2100");
%data=getData(data,rawDataPath+"2019-12-10_circ10s_160pwm_raw.xlsx","A100:U2100");
%data=getData(data,rawDataPath+"zero_raw.xlsx","A50:U400");
%data=getData(data,rawDataPath+"test2_raw.xlsx","A50:U100");
% data=getData(data,rawDataPath+"200negCirc.xlsx","A50:U2500");
% data=getData(data,rawDataPath+"200Circ.xlsx","A50:U2500");
% data=getData(data,rawDataPath+"200varspin.xlsx","A50:U5000");

data=getData2(data,rawDataPath+"200negCirc.xlsx",50,2900);
data=getData2(data,rawDataPath+"200Circ.xlsx",50,2900);
data=getData2(data,rawDataPath+"200varspin.xlsx",50,5900);

%% Create velocity data in robot frame of reference

data = [data, array2table(zeros(size(data,1),3), 'VariableNames',{'xVel','yVel','tVel'})];

for i=1:size(data,1)-1
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
        robotvel = [(data.rb5x(i+1)-data.rb5x(i))/1000,(data.rb5y(i+1)-data.rb5y(i))/1000]/(data.globalTime(i+1)-data.globalTime(i));
        %robotvel = [(data.rb5x(i+1)-data.rb5x(i))/1000,(data.rb5y(i+1)-data.rb5y(i))/1000]/0.02;
        RotationMatrix = [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)];
        robotXOrientation = [cos(pi/4) sin(pi/4)]*RotationMatrix;
        robotYOrientation = [cos(3*pi/4) sin(3*pi/4)]*RotationMatrix;
        robotxvel = dot(robotvel,robotXOrientation)/norm(robotXOrientation);
        robotyvel = dot(robotvel,robotYOrientation)/norm(robotYOrientation);
        robottvel = (theta2-theta1)/(data.globalTime(i+1)-data.globalTime(i));
        
        data.xVel(i)=robotxvel;
        data.yVel(i)=robotyvel;
        data.tVel(i)=robottvel;
end

%% Create matrices using every other variable

Input=[];
Output=[];

for i=20:2:size(data,1)-100
    
    pwm=[data.u2(i)-data.u4(i),0,0;0,data.u1(i)-data.u3(i),0;0,0,data.u1(i)+data.u2(i)+data.u3(i)+data.u4(i)];
    xk=[data.xVel(i),data.yVel(i),data.tVel(i)];
    Ei=[diag(xk),pwm,eye(3)];
    
    if data.tVel(i)<pi/3 && data.tVel(i+1)<pi/3
        Input = [Input;Ei];
        Output = [Output;data.xVel(i+1);data.yVel(i+1);data.tVel(i+1)];
    end
end

format long
P=lsqminnorm(Input,Output,'warn');
PP=Input\Output;


 norm(Input*P-Output)
 
 for i=21:2:size(data,1)-100
    
    pwm=[data.u2(i)-data.u4(i),0,0;0,data.u1(i)-data.u3(i),0;0,0,data.u1(i)+data.u2(i)+data.u3(i)+data.u4(i)];
    xk=[data.xVel(i),data.yVel(i),data.tVel(i)];
    Ei=[diag(xk),pwm,eye(3)];
   
    
   if data.tVel(i)<pi/3 && data.tVel(i+1)<pi/3
        Input = [Input;Ei];
        Output = [Output;data.xVel(i+1);data.yVel(i+1);data.tVel(i+1)];
    end
 end

 check = norm(Input*P-Output)
 
 
 a11=log(P(1))/0.02;
 a22=log(P(2))/0.02;
 a33=log(P(3))/0.02;
 
 b11=-P(4)*3.4*a11/P(1);
 b12=P(5)*3.4*a22/P(2);
 
 b21=P(6)*a33/(b11*(P(3)-1));
 b22=P(6)*a33/(b12*(P(3)-1));
 
 J=0.195/b22
 
 [a11 a22 a33 b11 b12 b21 b22 J];
 