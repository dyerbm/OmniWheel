%% Input Data from VICON Data Collection Script Output
function Process_from_raw_to_local_coord(filename)

output_file = "./Processed Data/" + filename + "_processed.xlsx";
data = xlsread("./Raw Data/" + filename + "_raw.xlsx");

globaltime = data(:,1);
m1x = data(:,3)./1000;
m1y = data(:,4)./1000;

m1x = movmean(m1x, 21);
m1y = movmean(m1y, 21);

m2x = data(:,6)./1000;
m2y = data(:,7)./1000;

m2x = movmean(m2x, 21);
m2y = movmean(m2y, 21);

m3x = data(:,9)./1000;
m3y = data(:,10)./1000;

m3x = movmean(m3x, 21);
m3y = movmean(m3y, 21);

m4x = data(:,12)./1000;
m4y = data(:,13)./1000;

m4x = movmean(m4x, 21);
m4y = movmean(m4y, 21);

m5x = data(:,15)./1000;
m5y = data(:,16)./1000;

m5x = movmean(m5x, 21);
m5y = movmean(m5y, 21);

u = data(:,18:21);

length = size(globaltime);

%% Calculate the vector needed to find the angle relative to the x axis
ver_diff = m4y - m5y;
hor_diff = m4x - m5x;

alpha = atan2(ver_diff, hor_diff);

alpha = movmean(alpha, 21);

Vx = zeros(length);
Vy = zeros(length);

V = zeros(length);
Vn = zeros(length);

omega = zeros(length);

for i = 2:length(1)
    
    beta = alpha(i) + pi/4; % add 45 degrees to the angle
    
    if i > 25
        
        Vx(i) = (m5x(i) - m5x(i-25))/(globaltime(i) - globaltime(i-25));
        Vy(i) = (m5y(i) - m5y(i-25))/(globaltime(i) - globaltime(i-25));
        omega(i) = (alpha(i) - alpha(i-25))/(globaltime(i) - globaltime(i-25));
        
        % Applying rotation matrix to shift to local coordinates
        V(i) = Vx(i) * cos(beta) + Vy(i) * sin(beta);
        Vn(i) = - Vx(i) * sin(beta) + Vy(i) * cos(beta);
           
    else
        
        Vx(i) = (m5x(i) - m5x(1))/(globaltime(i) - globaltime(1));
        Vy(i) = (m5y(i) - m5y(1))/(globaltime(i) - globaltime(1));
        omega(i) = (alpha(i) - alpha(1))/(globaltime(i) - globaltime(1));
        
        % Applying rotation matrix to shift to local coordinates
        V(i) = Vx(i) * cos(beta) + Vy(i) * sin(beta);
        Vn(i) = - Vx(i) * sin(beta) + Vy(i) * cos(beta);
        
    end
end


xlswrite(output_file, [globaltime, m5x, m5y, alpha, Vx, Vy, V, Vn, omega, u]);