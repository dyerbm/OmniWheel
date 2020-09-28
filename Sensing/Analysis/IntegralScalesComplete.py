#Calculate integral time and length scales assuming Taylor hypothesis
#using roof/street weather stations with 4Hz sampling
import random
import sys
import os
import numpy
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import datetime

########################################################################################################################
#Define averaging period in number of data points: minutes * seconds * samples
AverageSample=30*60*10

#Define averaging period for taking the integrals:
#this will be half of AverageSample since we have sliding window
WindowLength=int(AverageSample/2)

#Number of integration points: this is adjustable.
#This should be as large as possible
#Without making the integral time scale negative. Why?
NIntegral=1*60*10

#Sampling period [s]
dt=0.1

########################################################################################################################
#Define file names
fileName = "c:/Users/jdiro/Desktop/git/Sensing/testoutput.csv"

outputFileName="c:/Users/jdiro/Desktop/git/Sensing/IntegralScalesOutput.csv"

########################################################################################################################
#Load all data in a matrix
data4Hz = numpy.loadtxt(fileName, usecols=[0,1,2,3,4,5,7,8,9,10],skiprows=1,delimiter=",")

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

########################################################################################################################
#Calculate the number of samples
NSample=int(N4Hz/AverageSample)

#Define statistics and integral scales
yearavg=numpy.zeros((NSample,1))
monthavg=numpy.zeros((NSample,1))
dayavg=numpy.zeros((NSample,1))
timeHravg=numpy.zeros((NSample,1))
timeMinavg=numpy.zeros((NSample,1))
Uavg=numpy.zeros((NSample,1))
Vavg=numpy.zeros((NSample,1))
Wavg=numpy.zeros((NSample,1))
TSonicavg=numpy.zeros((NSample,1))
Uvar=numpy.zeros((NSample,1))
Vvar=numpy.zeros((NSample,1))
Wvar=numpy.zeros((NSample,1))
TSonicvar=numpy.zeros((NSample,1))
TauUU=numpy.zeros((NSample,1))
TauVV=numpy.zeros((NSample,1))
TauWW=numpy.zeros((NSample,1))
TauTT=numpy.zeros((NSample,1))
LUU=numpy.zeros((NSample,1))
LVV=numpy.zeros((NSample,1))
LWW=numpy.zeros((NSample,1))

#Define autocorrelation functions needed for each sample,
#i.e. R(s), where s is time shift
RUU=numpy.zeros((NIntegral,1))
RVV=numpy.zeros((NIntegral,1))
RWW=numpy.zeros((NIntegral,1))
RTT=numpy.zeros((NIntegral,1))

RUUAll=numpy.zeros((WindowLength,1))
RVVAll=numpy.zeros((WindowLength,1))
RWWAll=numpy.zeros((WindowLength,1))
RTTAll=numpy.zeros((WindowLength,1))

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
    Wmodel =numpy.polyfit(x,W,1)
    Wtrend = numpy.polyval(Wmodel,x)
    Wdetrended = W - Wtrend
    TSonic = TSonic4Hz[i*AverageSample:(i+1)*AverageSample]
    TSonicmodel = numpy.polyfit(x,TSonic,1)
    TSonictrend = numpy.polyval(TSonicmodel,x)
    TSonicdetrended = TSonic - TSonictrend

    #Calculate variances
    UVCovMatrix = numpy.cov(Udetrended, Vdetrended)
    UWCovMatrix = numpy.cov(Udetrended, Wdetrended)
    VWCovMatrix = numpy.cov(Vdetrended, Wdetrended)
    UTSonicCovMatrix = numpy.cov(Udetrended, TSonicdetrended)
    VTSonicCovMatrix = numpy.cov(Vdetrended, TSonicdetrended)
    WTSonicCovMatrix = numpy.cov(Wdetrended, TSonicdetrended)

    Uvar[i] = UVCovMatrix[0,0]
    Vvar[i] = UVCovMatrix[1,1]
    Wvar[i] = VWCovMatrix[1,1]
    TSonicvar[i] = UTSonicCovMatrix[1,1]

    #Calculate autocorrelation functions for a sliding window,
    #Half the size of each sample interval
    #Iterate over s, i.e. the time shift
    for s in range(0,NIntegral):
        #For each time shift s, iterate over the sliding window
        for k in range(0,WindowLength):
            RUUAll[k] = Udetrended[k]*Udetrended[k+s]
            RVVAll[k] = Vdetrended[k]*Vdetrended[k+s]
            RWWAll[k] = Wdetrended[k]*Wdetrended[k+s]
            RTTAll[k] = TSonicdetrended[k]*TSonicdetrended[k+s]
        #Calculate autocorrelation function for each s
        RUU[s] = numpy.mean(RUUAll)
        RVV[s] = numpy.mean(RVVAll)
        RWW[s] = numpy.mean(RWWAll)
        RTT[s] = numpy.mean(RTTAll)

    #Calculate integral scales
    TauUU[i] = numpy.sum(RUU*dt)/Uvar[i]
    TauVV[i] = numpy.sum(RVV*dt)/Vvar[i]
    TauWW[i] = numpy.sum(RWW*dt)/Wvar[i]
    TauTT[i] = numpy.sum(RTT*dt)/TSonicvar[i]

    LUU[i] = numpy.absolute(TauUU[i]*Uavg[i])
    LVV[i] = numpy.absolute(TauVV[i]*Vavg[i])
    LWW[i] = numpy.absolute(TauWW[i]*Wavg[i])

    #Print iteration number to see progress
    print('Iteration = ',i)

#Write data to file
outputFile = open(outputFileName, "w")
outputFile.write("#Times in Local Daylight Time - Subtract 1 hour for Local Standard Time \n")
outputFile.write("#0:Year \t 1:Month \t 2:Day \t 3:Hour \t 4:Minute \t \
    5:Uavg (m s-1) \t 6:Vavg (m s-1) \t 7:Wavg (m s-1) \t 8:TSonicavg (K) \t \
    9:TauUU (s) \t 10:TauVV (s) \t 11:TauWW (s) \t 12:TauTT (s) \
    13:LUU (m) \t 14:LVV (m) \t 15:LWW (m) \n")

for i in range(0,NSample):
    outputFile.write("%i \t %i \t %i \t %i \t %i \t \
    %f \t %f \t %f \t %f \t \
    %f \t %f \t %f \t %f \t \
    %f \t %f \t %f \n" \
    % (yearavg[i],monthavg[i],dayavg[i],timeHravg[i],timeMinavg[i], \
       Uavg[i], Vavg[i], Wavg[i], TSonicavg[i], \
       TauUU[i], TauVV[i], TauWW[i], TauTT[i], \
       LUU[i], LVV[i], LWW[i]))
outputFile.close()
