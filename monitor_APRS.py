
import pandas as pd
import datetime
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

myFmt = mdates.DateFormatter('%d-%H-%M') #Set the X axis for date HH:MM

df = pd.read_csv ("APRS.log")
#print(df.head())

df['packet_time']=pd.to_datetime(df.isotime,format='%Y-%m-%dT%H:%M:%SZ')
#print (df.tail())

#count packets by callsign
#packet_count=df.groupby('source').isotime.count()
#last_packet=df.groupby('source').packet_time.max()
#print (packet_count)

#Filter to W2MMD
#print 'W2MMD only'
df_mmd=df[df['source']=='W2MMD-13']
print 'W2MMD only'
print (df_mmd.tail())

#Filter to telemetry packets only
df_mmd1=df_mmd[df_mmd['symbol']=='/R']
print 'Telemetry only'
print (df_mmd1.tail())

#Remove extra columns
#print 'Filtered'
df_mmdc=df_mmd1[[8,20,22]]
#print (df_mmdc.tail())

#Parse out clubhouse temp from telemetry
print "Parsed temp"
print df_mmdc.telemetry.str.slice(10,12)
#df_mmdc['ctemp']=df_mmdc.telemetry.str.slice(10,12)
df_mmdc['ctemp']=df_mmdc.telemetry.str[10:12]
#print (df_mmdc.tail())

#filter out telemetry
df_mmdt=df_mmdc[[0,2,3]]
#print (df_mmdt.tail())

#drop NA values
df_mmdtx=df_mmdt[df_mmdt['ctemp'].notnull()]

#set index to date
df_mmdtx.set_index((df_mmdtx.packet_time), inplace=True)

#Filter to last 24 hours
print 'Final frame'
df_filt=df_mmdtx.last('24H')
#print (df_filt)

#Graph the temp
fig = plt.figure(dpi=100, figsize = (40,50))
ax1=fig.add_subplot(111)
ax1.grid(True)
#plt.plot(df_mmdtx.index,float(df_mmdtx.ctemp))
ax1.xaxis.set_major_formatter(myFmt) #Set the X axis format for dates
#plt.plot(df_mmdtx.index,df_mmdtx.ctemp.astype(int))
plt.plot(df_filt.index,df_filt.ctemp.astype(int))
#plt.plot(SampleTemp.index,SampleTemp)
plt.savefig('/var/www/html/curr_temp.jpg')
plt.show()
