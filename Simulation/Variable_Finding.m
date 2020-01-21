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
rawDataPath = "C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"
data=getData(data,rawDataPath+"spin150_raw.xlsx","A500:U1000");
data=getData(data,rawDataPath+"spinNeg150_raw.xlsx","A500:U1000");
data=getData(data,rawDataPath+"2019-12-10_circ10s_neg160pwm_raw.xlsx","A100:U1700");
data=getData(data,rawDataPath+"2019-12-10_circ10s_160pwm_raw.xlsx","A100:U1700");
%data=getData(data,rawDataPath+"zero_raw.xlsx","A50:U400");
%data=getData(data,rawDataPath+"test2_raw.xlsx","A50:U100");

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
    
   Input = [Input;Ei];
   Output = [Output;data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
end

format long
P=lsqminnorm(Input,Output,'warn');
PP=Input\Output;
lasso(Input,Output);


nVars=9;
% Ridge Regression

lambda = 100*ones(nVars,1); % Penalize each element by the same amount
R = chol(Input'*Input + diag(lambda));
wRR = R\(R'\(Input'*Output));
% 
% LASSO

lambda = 100*ones(nVars,1); % Penalize the absolute value of each element by the same amount
funObj = @(w)LogisticLoss(w,Input,Output); % Loss function that L1 regularization is applied to
w_init = wRR; % Initial value for iterative optimizer
fprintf('\nComputing LASSO Coefficients...\n');
wLASSO = L1General2_PSSgb(funObj,w_init,lambda);


fprintf('Number of non-zero variables in Ridge Regression solution: %d\n',nnz(wRR));
fprintf('Number of non-zero variables in LASSO solution: %d\n',nnz(wLASSO));


 norm(Input*wRR-Output)
 
 for i=21:2:size(data,1)-100
    
    vector_ca1 = [(data.rb4x(i) - data.rb5x(i))/1000, (data.rb4y(i) - data.rb5y(i))/1000];
    theta1 = atan(vector_ca1(2)/vector_ca1(1));
    
    vector_ca2 = [(data.rb4x(i+1) - data.rb5x(i+1))/1000, (data.rb4y(i+1) - data.rb5y(i+1))/1000];
    theta2 = atan(vector_ca2(2)/vector_ca2(1));
    
    pwm=[data.u2(i)-data.u4(i),0,0;0,data.u1(i)-data.u3(i),0;0,0,data.u1(i)+data.u2(i)+data.u3(i)+data.u4(i)];
    xk=[data.rb5x(i)/1000,data.rb5y(i)/1000,theta1];
    Ei=[diag(xk),pwm,eye(3)];
    
   Input = [Input;Ei];
   Output = [Output;data.rb5x(i+1)/1000;data.rb5y(i+1)/1000;theta2];
 end

 check = norm(Input*wRR-Output)