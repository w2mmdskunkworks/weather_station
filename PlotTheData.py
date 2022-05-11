#!/usr/bin/env python
#PlotTheData.py
#This creates the line graphs from the weather station
#Input is logcat.txt created by truncate_logfile.sh. If the program gives a date format error
#it's probably because the file isn't long enough to work with the tail statement in that script.
#Edit that script and reduce the tail line count to one that will work. You can check the line count in 
#that file with  cat logcat.txt | wc   the first number is the line count.

import os
import matplotlib
#matplotlib.use('Agg')
if os.environ.get('DISPLAY','') == '':
    print('no display found. Using non-interactive Agg backend')
    matplotlib.use('Agg')

from datetime import datetime, timedelta
from matplotlib import pyplot as plt
import matplotlib.dates as mdates
myFmt = mdates.DateFormatter('%m-%d') #Set the X axis for date HH:MM

import pandas as pd
from pandas import Series, DataFrame
import numpy as np

import csv
from datetime import datetime

#Load into dictionary - not using this now
#with open('headlog.txt') as f:
#	lines = list(csv.reader(f))
#header,values =lines[0], lines [1:]
#zip unpivots header row to get field names
#data_dict = {h: v for h, v in zip(header, zip (*values))}
#print (data_dict)
#convert dictionary to dataframe
#df = pd.DataFrame(data_dict)

print ('Reading file')
#Read the CSV file into a dataframe
df=pd.read_csv('/home/pi/logcat.txt',index_col=False)
print ('Frames read ' + str(len(df)))
print (df)

#print ('Create date')
#Create the concatenated date string
df['DateTimeStr'] = df['Date'] +'_' + df['Time']
print (df['DateTimeStr'])

#Convert it to date/time
df['SampleTime'] = [datetime.strptime(x,'%Y-%m-%d_%H:%M:%S') for x in df.DateTimeStr] 

#print ('Set index')
#Set as index
df.set_index(pd.DatetimeIndex(df.SampleTime), inplace=True)

#print ('Filter to 2 weeks')
#Filter to last n days
#print ('Filter')
df_filter = df.last('14D')

print ('Replace missing sensor values')
#Replace zero values when sensor is offline
df['WindDir'].replace(0.0,np.nan, inplace=True)
df['OutsideTemp'].replace(32.0,np.nan, inplace=True)
df['Visible'].replace(0,np.nan, inplace=True)

#print (df.index)
SampleInterval='30min'
MeanClubhouseTemp=df_filter.ClubhouseTemp.resample(SampleInterval).mean()
MeanOutsideTemp=df_filter.OutsideTemp.resample(SampleInterval).mean()
MeanWindSpeed=df_filter.WindSpeed.resample(SampleInterval).mean()
MeanOutsideHumidity=df_filter.OutsideHumidity.resample(SampleInterval).mean()
MeanOutsideTemp=df_filter.OutsideTemp.resample(SampleInterval).mean()
MeanVisible=df_filter.Visible.resample(SampleInterval).mean()
MeanAirQualityMetric=df_filter.AirQualityMetric.resample(SampleInterval).mean()
MeanRain=df_filter.Rain.resample(SampleInterval).mean()
MeanPressure=df_filter.BaroPressure.resample(SampleInterval).mean() 
MaxOccupied=df_filter.MotionSense.resample(SampleInterval).max() 

#print ('Starting plot')
#Plot the data			
fig = plt.figure(dpi=60, figsize = (20,10))
#plt.gcf().set_size_inches(10, 5)
#fig = plt.figure(dpi=60)
#fig = plt.figure(dpi=100)
#fig.set_figheight(15)
#fig.set_figwidth(15)
#plt.rcParams["figure.figsize"] = [5,5]
ax1=fig.add_subplot(511)
#plt.ylim(35,120)
ax1.xaxis.set_major_formatter(myFmt) #Set the X axis format for dates
ax1.plot(MeanClubhouseTemp.index,MeanClubhouseTemp,label='Clubhouse Temp',c='red')
ax1.plot(MeanOutsideTemp.index,MeanOutsideTemp,label='Outside Temp',c='blue')
plt.title('Clubhouse Telemetry')
plt.grid(True)
#Turn off X axis label for all but last plot
x_axis = ax1.axes.get_xaxis()
x_axis.set_visible(False)
#ax1.xlabel('Time')
plt.grid(True)
ax1.legend(loc='upper left')

#Add the second graph
ax2=fig.add_subplot(513)
ax2.xaxis.set_major_formatter(myFmt) #Set the X axis format for dates
ax2.plot(MeanWindSpeed.index,MeanWindSpeed,label='Windspeed',c='red')
ax2p=ax2.twinx()
ax2p.plot(MeanRain.index,MeanRain,label='Rain',c='blue')
ax2.legend(loc='upper left')
ax2p.legend(loc='upper right')
#Turn off X axis label for all but last plot
x_axis = ax2.axes.get_xaxis()
x_axis.set_visible(False)
ax2.grid(True)

#add the third graph
ax3=fig.add_subplot(514)
ax3.xaxis.set_major_formatter(myFmt) #Set the X axis format for dates
ax3.plot(MeanVisible.index,MeanVisible,label='Visible Light',c='red')
ax3p=ax3.twinx()
ax3p.plot(MaxOccupied.index,MaxOccupied,label='Occupied',c='blue')
ax3.legend(loc='upper left')
ax3p.legend(loc='upper right')
#Turn off X axis label for all but last plot
x_axis = ax3.axes.get_xaxis()
x_axis.set_visible(False)
ax3.grid(True)

#add the fourth graph
ax4=fig.add_subplot(515)
ax4.xaxis.set_major_formatter(myFmt) #Set the X axis format for dates
ax4.plot(MeanAirQualityMetric.index,MeanAirQualityMetric,label='Air Quality',c='red')
ax4.legend(loc='upper left')
#Turn off X axis label for all but last plot

ax4.grid(True)

#add the fifth graph
ax5=fig.add_subplot(512)
ax5.xaxis.set_major_formatter(myFmt) #Set the X axis format for dates
ax5p=ax5.twinx()
ax5p.plot(MaxOccupied.index,MeanOutsideHumidity,label='Humidity',c='blue')
ax5.plot(MeanPressure.index,MeanPressure,label='Pressure',c='red')
ax5.legend(loc='upper left')
ax5p.legend(loc='upper right')
#plt.ylim(29,31)
x_axis = ax5.axes.get_xaxis()
x_axis.set_visible(False)
ax5.grid(True)

#plt.savefig('/usr/share/apache2/icons/clubwx.png')
plt.savefig('/home/pi/clubwx.png')
plt.show ()
