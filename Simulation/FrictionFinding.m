data=[];
rawDataPath = "C:\Users\jdiro\Desktop\Git\Omniwheel VICON Code\Raw Data\"
data=getData(data,rawDataPath+"spin150_raw.xlsx","A500:U1000");
data=getData(data,rawDataPath+"spinNeg150_raw.xlsx","A500:U1000");
data=getData(data,rawDataPath+"2019-12-10_circ10s_neg160pwm_raw.xlsx","A100:U1700");
data=getData(data,rawDataPath+"2019-12-10_circ10s_160pwm_raw.xlsx","A100:U1700");