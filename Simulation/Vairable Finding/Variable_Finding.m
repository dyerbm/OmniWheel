%% Read in data
data=[];
rawDataPath = "C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\50Hz\";
%data=getData(data,rawDataPath+"spin150_raw.xlsx","A500:U1000");
%data=getData(data,rawDataPath+"spinNeg150_raw.xlsx","A500:U1000");
%data=getData(data,rawDataPath+"2019-12-10_circ10s_neg160pwm_raw.xlsx","A1000:U21000");
%data=getData(data,rawDataPath+"2019-12-10_circ10s_160pwm_raw.xlsx","A1000:U21000");
%data=getData(data,rawDataPath+"zero_raw.xlsx","A50:U400");
%data=getData(data,rawDataPath+"test2_raw.xlsx","A50:U1000");
% data=getData(data,rawDataPath+"200negCirc.xlsx","A50:U2500");
% data=getData(data,rawDataPath+"200Circ.xlsx","A50:U2500");
% data=getData(data,rawDataPath+"200varspin.xlsx","A50:U5000");


% data=getData2(data,rawDataPath+"200negCirc.xlsx",50,2500);
% data=getData2(data,rawDataPath+"200Circ.xlsx",50,2500);
% data=getData2(data,rawDataPath+"200varspin.xlsx",50,5000);
data=getData2(data,rawDataPath+"240negCirc_1.xlsx",50,6000);
data=getData2(data,rawDataPath+"240Circ_1.xlsx",50,6000);
data=getData2(data,rawDataPath+"240varspin_1.xlsx",50,12000);
% data=getData2(data,rawDataPath+"240negCirc_2.xlsx",50,6000);
% data=getData2(data,rawDataPath+"240Circ_2.xlsx",50,6000);
%data=getData2(data,rawDataPath+"240varspin_2.xlsx",50,6500);

%data.rb4y=MAF(data.rb4y,7)
%data.rb4x=MAF(data.rb4x,7)
%data.rb5y=MAF(data.rb5y,7)
%data.rb5x=MAF(data.rb5x,7)

for i=1:size(data,1)-1
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
        
        data.xVel(i)=robotxvel;
        data.yVel(i)=robotyvel;
        data.tVel(i)=robottvel;
        data.step(i)=data.globalTime(i+1)-data.globalTime(i);
end

%% Create matrices using every other variable

Input=[];
Output=[];
F=[];

for i=20:2:size(data,1)-1000
    
    vector_ca1 = [(data.rb4x(i) - data.rb5x(i))/1000, (data.rb4y(i) - data.rb5y(i))/1000];
    theta1 = atan(vector_ca1(2)/vector_ca1(1));
    
    vector_ca2 = [(data.rb4x(i+1) - data.rb5x(i+1))/1000, (data.rb4y(i+1) - data.rb5y(i+1))/1000];
    theta2 = atan(vector_ca2(2)/vector_ca2(1));
    
    %dvector = vector_ca2-vector_ca1;
    dvector = [(data.rb5x(i+1)-data.rb5x(i))/1000,(data.rb5y(i+1)-data.rb5y(i))/1000];
    
    RotationMatrix = [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)];
    robotXOrientation = [cos(pi/4) sin(pi/4)]*RotationMatrix;
    robotYOrientation = [cos(3*pi/4) sin(3*pi/4)]*RotationMatrix;
    x1 = dot(vector_ca1(1:2),robotXOrientation)/norm(robotXOrientation);
    x2 = dot(vector_ca2(1:2),robotXOrientation)/norm(robotXOrientation);
    y1 = dot(vector_ca1(1:2),robotYOrientation)/norm(robotYOrientation);
    y2 = dot(vector_ca2(1:2),robotYOrientation)/norm(robotYOrientation);
    
    x_diff = dot(dvector,robotXOrientation)/norm(robotXOrientation);
    y_diff = dot(dvector,robotYOrientation)/norm(robotYOrientation);
    
    pwm=[data.u2(i)-data.u4(i),0,0;
        0,data.u1(i)-data.u3(i),0;
        0,0,data.u1(i)+data.u2(i)+data.u3(i)+data.u4(i)];
%     pwm = [data.u1(i) data.u2(i) data.u3(i) data.u4(i) 0 0 0 0 0 0 0 0;
%         0 0 0 0 data.u1(i) data.u2(i) data.u3(i) data.u4(i) 0 0 0 0;
%         0 0 0 0 0 0 0 0 data.u1(i) data.u2(i) data.u3(i) data.u4(i)];
    xk=[data.rb5x(i)/1000,data.rb5y(i)/1000,theta1];
    %xk=[x_diff,y_diff,theta1]
    Ei=[diag(xk),pwm];
    %Ei=[pwm];
    
    if abs(data.step(i)-0.02)<0.0001 && abs(data.tVel(i))<pi/6 && abs(data.tVel(i+1))<pi/6 && abs(data.xVel(i)-data.xVel(i+1))<1.5 && abs(data.yVel(i)-data.yVel(i+1))<1.5
        data.xVel(i);
        xk(1)-data.rb5x(i+1)/1000;
        y_diff;
        F = [F;0.0197*data.xVel(i)+8.12e-6;0.02047*data.yVel(i)-3.946e-6;data.tVel(i)*0.0177-1.515e-4];
        Input = [Input;Ei];
        %Output = [Output; data.rb5x(i+1)/1000+xk(;data.rb5y(i+1)/1000;theta2];
        Output = [Output; xk(1)+x_diff;xk(2)+y_diff;theta2];
        %Output = [Output; x_diff;y_diff;theta2-theta1];
    end
end

format long
P=lsqminnorm(Input,Output,'warn');
PP=lsqminnorm(Input,Output-F,'warn');


norm(Input*P-Output)
 
% Input=[];
% Output=[];
 
% for i=21:2:size(data,1)-1000
%     
%     vector_ca1 = [(data.rb4x(i) - data.rb5x(i))/1000, (data.rb4y(i) - data.rb5y(i))/1000];
%     theta1 = atan(vector_ca1(2)/vector_ca1(1));
%     
%     vector_ca2 = [(data.rb4x(i+1) - data.rb5x(i+1))/1000, (data.rb4y(i+1) - data.rb5y(i+1))/1000];
%     theta2 = atan(vector_ca2(2)/vector_ca2(1));
%     
%     RotationMatrix = [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)];
%     robotXOrientation = [cos(pi/4) sin(pi/4)]*RotationMatrix;
%     robotYOrientation = [cos(3*pi/4) sin(3*pi/4)]*RotationMatrix;
%     x1 = dot(vector_ca1(1:2),robotXOrientation)/norm(robotXOrientation);
%     x2 = dot(vector_ca2(1:2),robotXOrientation)/norm(robotXOrientation);
%     y1 = dot(vector_ca1(1:2),robotYOrientation)/norm(robotYOrientation);
%     y2 = dot(vector_ca2(1:2),robotYOrientation)/norm(robotYOrientation);
%     
%     pwm=[-data.u2(i)+data.u4(i),0,0;0,data.u1(i)-data.u3(i),0;0,0,data.u1(i)+data.u2(i)+data.u3(i)+data.u4(i)];
%     xk=[x1,y1,theta1];
%     Ei=[diag(xk),pwm,eye(3)];
%     
%     if abs(data.step(i)-0.02)<0.0001 && abs(data.tVel(i))<pi/3 && abs(data.tVel(i+1))<pi/3 && abs(data.xVel(i)-data.xVel(i+1))<3 && abs(data.yVel(i)+data.yVel(i+1))<3
%         Input = [Input;Ei];
%         Output = [Output;x2;y2;theta2];
%     end
%  end
% 
%  check = norm(Input*P-Output)
 
 
 a11=log(P(1))/0.02;
 a22=log(P(2))/0.02;
 a33=log(P(3))/0.02;
 
 b11=-P(4)*3.4*a11/P(1);
 b12=P(5)*3.4*a22/P(2);
 
 b21=P(6)*a33/(b11*(P(3)-1));
 b22=P(6)*a33/(b12*(P(3)-1));
 
 J=0.195/b21
 
 [a11 a22 a33 b11 b12 b21 b22 J];
 