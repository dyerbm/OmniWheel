#Read and plot one-point turbulent statistics
import random
import sys
import os
import numpy
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import datetime

#Define file names
fileName = "c:/Users/jdiro/Desktop/git/Sensing/IntegralScalesOutput.csv"

#Load all data in a matrix
data = numpy.loadtxt(fileName)

yearavg=data[:,0]
monthavg=data[:,1]
dayavg=data[:,2]
timeHravg=data[:,3]
timeMinavg=data[:,4]
Uavg=data[:,5]
Vavg=data[:,6]
Wavg=data[:,7]
TSonicavg=data[:,8]
TauUU=data[:,9]
TauVV=data[:,10]
TauWW=data[:,11]
TauTT=data[:,12]
LUU=data[:,13]
LVV=data[:,14]
LWW=data[:,15]

N=len(yearavg)

#Create date time vectors
#First convert to seconds then to YYYY-MM-DD-HH-mm-ss
dateTimeSec=numpy.zeros((N,1))
for i in range(0,N):
    dateTimeSec[i]=datetime.datetime(int(yearavg[i]), int(monthavg[i]), \
        int(dayavg[i]), int(timeHravg[i]), int(timeMinavg[i]), 0).timestamp()

dateTime = [datetime.datetime.fromtimestamp(val) for val in dateTimeSec]

#Plot the results

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, TauUU, 'ko', markersize=4)
plt.ylabel('TauUU [s]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, TauUU, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=12)
plt.ylabel('TauUU [s]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, TauVV, 'ko', markersize=4)
plt.ylabel('TauVV [s]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, TauVV, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=12)
plt.ylabel('TauVV [s]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, TauWW, 'ko', markersize=4)
plt.ylabel('TauWW [s]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, TauWW, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=12)
plt.ylabel('TauWW [s]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, TauTT, 'ko', markersize=4)
plt.ylabel('TauTT [s]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, TauTT, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=12)
plt.ylabel('TauTT [s]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, LUU, 'ko', markersize=4)
plt.ylabel('LUU [m]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, LUU, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=12)
plt.ylabel('LUU [m]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, LVV, 'ko', markersize=4)
plt.ylabel('LVV [m]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, LVV, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=12)
plt.ylabel('LVV [m]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, LWW, 'ko', markersize=4)
plt.ylabel('LWW [m]',fontsize=12)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, LWW, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=12)
plt.ylabel('LWW [m]',fontsize=12)
plt.tight_layout()
fig.show()


plt.show()