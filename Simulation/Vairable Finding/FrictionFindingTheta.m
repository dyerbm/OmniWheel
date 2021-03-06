
results = []
Velocity = []

%% for negative spins
for i=-250:10:-50
    
    data=[];
    rawDataPath = "C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\spinData\";
    data=getData(data,rawDataPath+int2str(i)+"spin_raw.xlsx","A200:U700");
    %data=getData(data,rawDataPath+"spinNeg150_raw.xlsx","A500:U1000");
    %data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"2019-12-10_circ10s_neg160pwm_raw.xlsx","A100:U1700");
    %data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"2019-12-10_circ10s_160pwm_raw.xlsx","A100:U1700");
    %data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"250circ2_raw.xlsx","A1400:U2700");

    %Find Friction

    Input=[];
    Output=[];
    vels=[];

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
       
       % find velocity
       if i<400
            dtheta=theta2-theta1
            if dtheta>0
                dtheta=dtheta
            else
               vels = [vels; (dtheta)/(data.globalTime(i+1)-data.globalTime(i))];
            end
       end
    end

    format long
    P=lsqminnorm(Input,Output,'warn');
    
    results=[results; P(9)];
    Velocity = [Velocity; mean(vels)];
end

%% for positive spins
for i=50:10:250
    
    data=[];
    rawDataPath = "C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\spinData\";
    data=getData(data,rawDataPath+int2str(i)+"spin_raw.xlsx","A200:U700");
    %data=getData(data,rawDataPath+"spinNeg150_raw.xlsx","A500:U1000");
    %data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"2019-12-10_circ10s_neg160pwm_raw.xlsx","A100:U200");
    %data=getData(data,"C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"+"2019-12-10_circ10s_160pwm_raw.xlsx","A100:U200");

    %Find Friction

    Input=[];
    Output=[];
    vels=[];

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
       
       if i<400
        dtheta=theta2-theta1
        if dtheta<0
            dtheta=dtheta
        else
           vels = [vels; (dtheta)/(data.globalTime(i+1)-data.globalTime(i))];
        end
       end
    end

    format long
    P=lsqminnorm(Input,Output,'warn');
    
    results=[results; P(9)];
    Velocity = [Velocity; mean(vels)];
end

results
Velocity


