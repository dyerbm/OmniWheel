#Calculate one-point turbulent statistics using roof/street weather stations with 4Hz sampling
import random
import sys
import os
import numpy
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import datetime

#Define averaging period in number of data points: minutes * seconds * samples
AverageSample=30*60*10

#Define file names
fileName = "c:/Users/jdiro/Desktop/git/Sensing/testoutput.csv"

outputFileNameOnePointStatistics="c:/Users/jdiro/Desktop/git/Sensing/OnePointStatisticsOutput.txt"

#Load all data in a matrix
data4Hz = numpy.loadtxt(fileName, usecols=[0,1,2,3,4,5,7,8,9,10], delimiter=",", skiprows=1)

year4Hz=data4Hz[:,0]
month4Hz=data4Hz[:,1]
day4Hz=data4Hz[:,2]
timeHr4Hz=data4Hz[:,3]
timeMin4Hz=data4Hz[:,4]
timeSec4Hz=data4Hz[:,5]
U4Hz=data4Hz[:,6]
V4Hz=data4Hz[:,7]
W4Hz=data4Hz[:,8]
TSonic4Hz=data4Hz[:,9]

N4Hz=numpy.size(year4Hz)

#Calculate the number of samples and then detrend data
NSample=int(N4Hz/AverageSample)

#Define statistics, in addition to U and V, define S for along wind horizontal wind speed
yearavg=numpy.zeros((NSample,1))
monthavg=numpy.zeros((NSample,1))
dayavg=numpy.zeros((NSample,1))
timeHravg=numpy.zeros((NSample,1))
timeMinavg=numpy.zeros((NSample,1))
Uavg=numpy.zeros((NSample,1))
Vavg=numpy.zeros((NSample,1))
Savg=numpy.zeros((NSample,1))
Wavg=numpy.zeros((NSample,1))
TSonicavg=numpy.zeros((NSample,1))
Uvar=numpy.zeros((NSample,1))
Vvar=numpy.zeros((NSample,1))
Wvar=numpy.zeros((NSample,1))
TSonicvar=numpy.zeros((NSample,1))
k=numpy.zeros((NSample,1))
UVcov=numpy.zeros((NSample,1))
UWcov=numpy.zeros((NSample,1))
VWcov=numpy.zeros((NSample,1))
UTSoniccov=numpy.zeros((NSample,1))
VTSoniccov=numpy.zeros((NSample,1))
WTSoniccov=numpy.zeros((NSample,1))

for i in range(0,NSample):
    #Calculate year, month, day, hour, and minute for each sample
    yearavg[i] = numpy.mean(year4Hz[i*AverageSample:(i+1)*AverageSample])
    monthavg[i] = numpy.mean(month4Hz[i*AverageSample:(i+1)*AverageSample])
    dayavg[i] = numpy.mean(day4Hz[i*AverageSample:(i+1)*AverageSample])
    timeHravg[i] = numpy.mean(timeHr4Hz[i*AverageSample:(i+1)*AverageSample])
    timeMinavg[i] = numpy.mean(timeMin4Hz[i*AverageSample:(i+1)*AverageSample])+1

    #Calculate averages
    Uavg[i] = numpy.mean(U4Hz[i*AverageSample:(i+1)*AverageSample])
    Vavg[i] = numpy.mean(V4Hz[i*AverageSample:(i+1)*AverageSample])
    Wavg[i] = numpy.mean(W4Hz[i*AverageSample:(i+1)*AverageSample])
    TSonicavg[i] = numpy.mean(TSonic4Hz[i*AverageSample:(i+1)*AverageSample])
    Savg[i] = numpy.mean(numpy.sqrt(U4Hz[i*AverageSample:(i+1)*AverageSample] ** 2 + \
                            V4Hz[i*AverageSample:(i+1)*AverageSample] ** 2))

    #Define a vector for the number of data points in each sample
    x = [j for j in range(0, AverageSample)]
    #Detrend each sample, i.e. remove a straight line fit from the sample
    U = U4Hz[i*AverageSample:(i+1)*AverageSample]
    Umodel = numpy.polyfit(x,U,1)
    Utrend = numpy.polyval(Umodel,x)
    Udetrended = U - Utrend
    V = V4Hz[i*AverageSample:(i+1)*AverageSample]
    Vmodel = numpy.polyfit(x,V,1)
    Vtrend = numpy.polyval(Vmodel,x)
    Vdetrended = V - Vtrend
    W = W4Hz[i*AverageSample:(i+1)*AverageSample]
    Wmodel = numpy.polyfit(x,W,1)
    Wtrend = numpy.polyval(Wmodel,x)
    Wdetrended = W - Wtrend
    TSonic =  TSonic4Hz[i*AverageSample:(i+1)*AverageSample]
    TSonicmodel = numpy.polyfit(x, TSonic,1)
    TSonictrend = numpy.polyval( TSonicmodel,x)
    TSonicdetrended =  TSonic -  TSonictrend

    #Calculate variances, and covariances
    UVCovMatrix = numpy.cov(Udetrended, Vdetrended)
    UWCovMatrix = numpy.cov(Udetrended, Wdetrended)
    VWCovMatrix = numpy.cov(Vdetrended, Wdetrended)
    UTSonicCovMatrix = numpy.cov(Udetrended, TSonicdetrended)
    VTSonicCovMatrix = numpy.cov(Vdetrended, TSonicdetrended)
    WTSonicCovMatrix = numpy.cov(Wdetrended, TSonicdetrended)

    Uvar[i] = UVCovMatrix[0,0]
    Vvar[i] = UVCovMatrix[1,1]
    Wvar[i] = UWCovMatrix[1,1]
    TSonicvar[i] = UTSonicCovMatrix[1,1]
    k[i] = 1/2*(Uvar[i]+Vvar[i]+Wvar[i])

    UVcov[i] = UVCovMatrix[0,1]
    UWcov[i] = UWCovMatrix[0,1]
    VWcov[i] = VWCovMatrix[0,1]
    UTSoniccov[i] = UTSonicCovMatrix[0,1]
    VTSoniccov[i] = VTSonicCovMatrix[0,1]
    WTSoniccov[i] = WTSonicCovMatrix[0,1]

#Write data to file
outputFile = open(outputFileNameOnePointStatistics, "w")
outputFile.write("#Times in Local Daylight Time - Subtract 1 hour for Local Standard Time \n")
outputFile.write("#0:Year \t 1:Month \t 2:Day \t 3:Hour \t 4:Minute \t \
    5:Uavg (m s-1) \t 6:Vavg (m s-1) \t 7:Savg (m s-1) \t 8:Wavg (m s-1) \t 9:TSonicavg (K) \t \
    10:Uvar (m2 s-2) \t 11:Vvar (m2 s-2) \t 12:Wvar (m2 s-2) \t 13:TSonicvar (K2) \t 14:k (m2 s-2) \t \
    15:UVcov (m2 s-2) \t 16:UWcov (m2 s-2) \t 17:VWcov (m2 s-2) \t \
    18:UTSoniccov (Km s-1) \t 19:VTSoniccov (Km s-1) \t 20:WTSoniccov (Km s-1) \n")

for i in range(0,NSample):
    outputFile.write("%i \t %i \t %i \t %i \t %i \t \
    %f \t %f \t %f \t %f \t %f \t \
    %f \t %f \t %f \t %f \t %f \t \
    %f \t %f \t %f \
    %f \t %f \t %f \n" \
    % (yearavg[i],monthavg[i],dayavg[i],timeHravg[i],timeMinavg[i],Uavg[i],Vavg[i],Savg[i],Wavg[i],
    TSonicavg[i],Uvar[i],Vvar[i],Wvar[i],TSonicvar[i],k[i],UVcov[i],UWcov[i],
    VWcov[i],UTSoniccov[i],VTSoniccov[i],WTSoniccov[i]))
outputFile.close()