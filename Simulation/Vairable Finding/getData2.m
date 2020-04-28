function output = getData2(output,filePath,start,stop)
    %% Import the data
    [~, ~, raw] = xlsread(filePath,'Sheet1');
    raw = raw(start:stop,:);

    %% Create output variable
    data = reshape([raw{:}],size(raw));

    %% Create table
    Circ = table;

    %% Allocate imported array to column variable names
    Circ.globalTime = data(:,1);
    Circ.timestep = data(:,2);
    Circ.rb1x = data(:,3);
    Circ.rb1y = data(:,4);
    Circ.rb1z = data(:,5);
    Circ.rb2x = data(:,6);
    Circ.rb2y = data(:,7);
    Circ.rb2z = data(:,8);
    Circ.rb3x = data(:,9);
    Circ.rb3y = data(:,10);
    Circ.rb3z = data(:,11);
    Circ.rb4x = data(:,12);
    Circ.rb4y = data(:,13);
    Circ.rb4z = data(:,14);
    Circ.rb5x = data(:,15);
    Circ.rb5y = data(:,16);
    Circ.rb5z = data(:,17);
    Circ.u1 = data(:,18);
    Circ.u2 = data(:,19);
    Circ.u3 = data(:,20);
    Circ.u4 = data(:,21);

    %% Clear temporary variables
    clearvars data raw;

    output=[output; Circ];

end