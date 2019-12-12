%% Setup the Import Options
opts = spreadsheetImportOptions("NumVariables", 21);

% Specify sheet and range
opts.Sheet = "Sheet1";
opts.DataRange = "A2:U2000";

% Specify column names and types
opts.VariableNames = ["globalTime", "timeNow", "rb1x", "rb1y", "rb1z", "rb2x", "rb2y", "rb2z", "rb3x", "rb3y", "rb3z", "rb4x", "rb4y", "rb4z", "rb5x", "rb5y", "rb5z", "u1", "u2", "u3", "u4"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Import the data
data = readtable("C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\2019-12-10_circ10s_160pwm_raw.xlsx", opts, "UseExcel", false);


%% Clear temporary variables
clear opts

%% Setup the Import Options
opts = spreadsheetImportOptions("NumVariables", 21);

% Specify sheet and range
opts.Sheet = "Sheet1";
opts.DataRange = "A2:U2000";

% Specify column names and types
opts.VariableNames = ["globalTime", "timeNow", "rb1x", "rb1y", "rb1z", "rb2x", "rb2y", "rb2z", "rb3x", "rb3y", "rb3z", "rb4x", "rb4y", "rb4z", "rb5x", "rb5y", "rb5z", "u1", "u2", "u3", "u4"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Import the data
data = [data; readtable("C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\2019-12-10_circ10s_neg160pwm_raw.xlsx", opts, "UseExcel", false)];


%% Clear temporary variables
clear opts

%% Create matrices using every other variable

Input=[];
Output=[];

for i=21:2:size(data,1)-100
    
    vector_ca1 = [(data.rb4x(i) - data.rb5x(i))/1000, (data.rb4y(i) - data.rb5y(i))/1000];
    theta1 = atan(vector_ca1(2)/vector_ca1(1));
    
    vector_ca2 = [(data.rb4x(i+1) - data.rb5x(i+1))/1000, (data.rb4y(i+1) - data.rb5y(i+1))/1000];
    theta2 = atan(vector_ca2(2)/vector_ca2(1));
    
    pwm=[0,data.u2(i),0,-data.u4(i);data.u1(i),0,-data.u3(i),0;data.u1(i),data.u2(i),data.u3(i),data.u4(i)];
    xk=[data.rb5x(i)/1000,data.rb5y(i)/1000,theta1];
    Ei=[diag(xk),pwm,eye(3)];
    
   Input = [Input;Ei];
   Output = [Output;data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
end

format long
P=lsqminnorm(Input,Output,1e-20,'warn')
PP=Input\Output;

 norm(Input*P-Output)
 
 for i=21:1:size(data,1)-100
    
    vector_ca1 = [(data.rb4x(i) - data.rb5x(i))/1000, (data.rb4y(i) - data.rb5y(i))/1000];
    theta1 = atan(vector_ca1(2)/vector_ca1(1));
    
    vector_ca2 = [(data.rb4x(i+1) - data.rb5x(i+1))/1000, (data.rb4y(i+1) - data.rb5y(i+1))/1000];
    theta2 = atan(vector_ca2(2)/vector_ca2(1));
    
    pwm=[0,data.u2(i),0,-data.u4(i);data.u1(i),0,-data.u3(i),0;data.u1(i),data.u2(i),data.u3(i),data.u4(i)];
    xk=[data.rb5x(i)/1000,data.rb5y(i)/1000,theta1];
    Ei=[diag(xk),pwm,eye(3)];
    
   Input = [Input;Ei];
   Output = [Output;data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
 end

 check = norm(Input*P-Output)