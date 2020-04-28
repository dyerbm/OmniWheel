function averaged = MAF(data,windowSize)
    b = (1/windowSize)*ones(1,windowSize);
    a = 1;
    
    averaged = filter(b,a,data);
end