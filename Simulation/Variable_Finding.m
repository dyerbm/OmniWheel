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


data=getData2(data,rawDataPath+"200negCirc.xlsx",50,2500);
data=getData2(data,rawDataPath+"200Circ.xlsx",50,2500);
data=getData2(data,rawDataPath+"200varspin.xlsx",50,5000);
data=getData2(data,rawDataPath+"240negCirc_1.xlsx",50,6000);
data=getData2(data,rawDataPath+"240Circ_1.xlsx",50,6000);
data=getData2(data,rawDataPath+"240varspin_1.xlsx",50,12000);
data=getData2(data,rawDataPath+"240negCirc_2.xlsx",50,6000);
data=getData2(data,rawDataPath+"240Circ_2.xlsx",50,6000);
data=getData2(data,rawDataPath+"240varspin_2.xlsx",50,6500);

data.rb4y=MAF(data.rb4y,7)
data.rb4x=MAF(data.rb4x,7)
data.rb5y=MAF(data.rb5y,7)
data.rb5x=MAF(data.rb5x,7)
%% Create matrices using every other variable

Input=[];
Output=[];

for i=20:2:size(data,1)-100
    
    vector_ca1 = [(data.rb4x(i) - data.rb5x(i))/1000, (data.rb4y(i) - data.rb5y(i))/1000];
    theta1 = atan(vector_ca1(2)/vector_ca1(1));
    
    vector_ca2 = [(data.rb4x(i+1) - data.rb5x(i+1))/1000, (data.rb4y(i+1) - data.rb5y(i+1))/1000];
    theta2 = atan(vector_ca2(2)/vector_ca2(1));
    
    pwm=[data.u2(i)-data.u4(i),0,0;0,data.u1(i)-data.u3(i),0;0,0,data.u1(i)+data.u2(i)+data.u3(i)+data.u4(i)];
    xk=[data.rb5x(i)/1000,data.rb5y(i)/1000,theta1];
    Ei=[diag(xk),pwm,eye(3)];
    
    if abs(theta2-theta1)<pi/3
        Input = [Input;Ei];
        Output = [Output;data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
    end
end

format long
P=lsqminnorm(Input,Output,'warn');
PP=Input\Output;


 norm(Input*P-Output)
 
 for i=21:2:size(data,1)-100
    
    vector_ca1 = [(data.rb4x(i) - data.rb5x(i))/1000, (data.rb4y(i) - data.rb5y(i))/1000];
    theta1 = atan(vector_ca1(2)/vector_ca1(1));
    
    vector_ca2 = [(data.rb4x(i+1) - data.rb5x(i+1))/1000, (data.rb4y(i+1) - data.rb5y(i+1))/1000];
    theta2 = atan(vector_ca2(2)/vector_ca2(1));
    
    pwm=[data.u2(i)-data.u4(i),0,0;0,data.u1(i)-data.u3(i),0;0,0,data.u1(i)+data.u2(i)+data.u3(i)+data.u4(i)];
    xk=[data.rb5x(i)/1000,data.rb5y(i)/1000,theta1];
    Ei=[diag(xk),pwm,eye(3)];
   
    
   if abs(theta2-theta1)<pi/2
        Input = [Input;Ei];
        Output = [Output;data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
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
 
 J=0.195/b21
 
 [a11 a22 a33 b11 b12 b21 b22 J];
 