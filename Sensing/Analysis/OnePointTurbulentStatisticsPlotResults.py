#Read and plot one-point turbulent statistics
import random
import sys
import os
import numpy
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import datetime

#Define file names
fileName = "c:/Users/jdiro/Desktop/git/Sensing/Analysis/OnePointStatisticsOutput.txt"

#Load all data in a matrix
data = numpy.loadtxt(fileName)

yearavg=data[:,0]
monthavg=data[:,1]
dayavg=data[:,2]
timeHravg=data[:,3]
timeMinavg=data[:,4]
Uavg=data[:,5]
Vavg=data[:,6]
Savg=data[:,7]
Wavg=data[:,8]
TSonicavg=data[:,9]
Uvar=data[:,10]
Vvar=data[:,11]
Wvar=data[:,12]
TSonicvar=data[:,13]
k=data[:,14]
UVcov=data[:,15]
UWcov=data[:,16]
VWcov=data[:,17]
UTSoniccov=data[:,18]
VTSoniccov=data[:,19]
WTSoniccov=data[:,20]

N=len(yearavg)

#Find maximum temperature variation
DeltaT=numpy.max(TSonicavg)-numpy.min(TSonicavg)

#Create date time vectors
#First convert to seconds then to YYYY-MM-DD-HH-mm-ss
dateTimeSec=numpy.zeros((N,1))
for i in range(0,N):
    dateTimeSec[i]=datetime.datetime(int(yearavg[i]), int(monthavg[i]), \
        int(dayavg[i]), int(timeHravg[i]), int(timeMinavg[i]), 0).timestamp()

dateTime = [datetime.datetime.fromtimestamp(val) for val in dateTimeSec]

#Plot the results

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, Uavg, 'ko', markersize=4)
plt.ylabel('Uavg [m s-1]',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, Uavg, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('Uavg [m s-1]',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, Vavg, 'ko', markersize=4)
plt.ylabel('Vavg [m s-1]',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, Vavg, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('Vavg [m s-1]',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, Savg, 'ko', markersize=4)
plt.ylabel('(Uavg**2+Vavg**2)**0.5 [m s-1]',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, Savg, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('(Uavg**2+Vavg**2)**0.5 [m s-1]',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, Wavg, 'ko', markersize=4)
plt.ylabel('Wavg [m s-1]',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, Wavg, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('Wavg [m s-1]',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, TSonicavg, 'ko', markersize=4)
plt.ylabel('T [K]',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, TSonicavg, 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('T [K]',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, Uvar/(Savg**2), 'ko', markersize=4)
plt.ylabel('Uvar/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, Uvar/(Savg**2), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('Uvar/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, Vvar/(Savg**2), 'ko', markersize=4)
plt.ylabel('Vvar/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, Vvar/(Savg**2), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('Vvar/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, Wvar/(Savg**2), 'ko', markersize=4)
plt.ylabel('Wvar/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, Wvar/(Savg**2), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('Wvar/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, TSonicvar/(DeltaT**2), 'ko', markersize=4)
plt.ylabel('Tvar/(Delta T)**2',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, TSonicvar/(DeltaT**2), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('Tvar/(Delta T)**2',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, k/(Savg**2), 'ko', markersize=4)
plt.ylabel('k/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, k/(Savg**2), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('k/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, UVcov/(Savg**2), 'ko', markersize=4)
plt.ylabel('UVcov/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, UVcov/(Savg**2), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('UVcov/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, UWcov/(Savg**2), 'ko', markersize=4)
plt.ylabel('UWcov/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, UWcov/(Savg**2), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('UWcov/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, VWcov/(Savg**2), 'ko', markersize=4)
plt.ylabel('VWcov/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, VWcov/(Savg**2), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('VWcov/(Uavg**2+Vavg**2)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, UTSoniccov/(Savg*DeltaT), 'ko', markersize=4)
plt.ylabel('UTcov/((Uavg**2+Vavg**2)**0.5 Delta T)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, UTSoniccov/(Savg*DeltaT), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('UTcov/((Uavg**2+Vavg**2)**0.5 Delta T))',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, VTSoniccov/(Savg*DeltaT), 'ko', markersize=4)
plt.ylabel('VTcov/((Uavg**2+Vavg**2)**0.5 Delta T)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, VTSoniccov/(Savg*DeltaT), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('VTcov/((Uavg**2+Vavg**2)**0.5 Delta T)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(7,3))
plt.plot(dateTime, WTSoniccov/(Savg*DeltaT), 'ko', markersize=4)
plt.ylabel('WTcov/((Uavg**2+Vavg**2)**0.5 Delta T)',fontsize=10)
plt.tight_layout()
fig.show()

fig = plt.figure(figsize=(3,3))
plt.plot(timeHravg, WTSoniccov/(Savg*DeltaT), 'ko', markersize=4)
plt.xlabel('Local Daylight Time [hr]', fontsize=10)
plt.ylabel('WTcov/((Uavg**2+Vavg**2)**0.5 Delta T)',fontsize=10)
plt.tight_layout()
fig.show()

plt.show()