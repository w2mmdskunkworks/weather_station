import csv
from datetime import datetime
import os

WXdates = []
WXtimes=[]
windspeed=[]
windgust=[]
winddir=[]
baropressure=[]
outsidetemp=[]
clubhousetemp=[]
humidity=[]
rain=[]
motionsense=[]
lightningcount=[]
airqualitymetric=[]
outside_crc = []

filename = '/home/pi/APRSlog.txt'
with open (filename) as f:
	reader=csv.reader(f)
	header_row = next(reader)
	#print (header_row)
	for row in reader:
		WXdates.append(row[0])
		WXtimes.append(row[1])
		windspeed.append(row[2])
		winddir.append(row[3])
		windgust.append(row[19])
		baropressure.append(row[16])
		motionsense.append(row[17])
		outside_crc.append(row[18])
		outsidetemp.append(row[7])
		clubhousetemp.append(row[6])
		humidity.append(row[8])
		rain.append(row[5])
		lightningcount.append(row[13])
		airqualitymetric.append(row[14])
	
#If bad CRC, don't use the value	- doesn't work	
#if float(outside_crc[-1]) != 1.0:
#	outsidetemp[-1] = None
#	humidity[-1]=None

#If outside temp isn't reporting use inside temp			
#if float(outsidetemp[-1]) == 32.0: 
#	outsidetemp[-1] = clubhousetemp[-1]
	
#print (WXdates[1:10])
#print len(WXdates)
#Get last element of lists
print (WXdates[-1],WXtimes[-1],windspeed[-1],outsidetemp[-1],humidity[-1])

#create the beacon string
#print ('@'+str(WXtimes[-1]).replace(':','')+'z3971.60N/07521.0W_000/'+str(int(round(float(windspeed[-1])))).rjust(3,'0')+'g'+str(int(round(float(windgust[-1])))).rjust(3,'0') +'b'+str(int(round(float(baropressure[-1])))).rjust(3,'0')  +'wRSW')
#print ('000/'+str(int(round(float(windspeed[-1])))).rjust(3,'0')+'g'+str(int(round(float(windgust[-1])))).rjust(3,'0') +'t'+ str(int(round(float(outsidetemp[-1])))).rjust(3,'0') +'r'+ str(int(round(float(rain[-1])))).rjust(3,'0') +'h'+ str(int(round(float(humidity[-1])))).rjust(2,'0')  +'b'+str(int(round(float(baropressure[-1])/.00295301))).rjust(5,'0'))

#Skip writing beacon text if CRC is bad
if float(outside_crc[-1]) != -1.0:
#write weather beacon file
	if os.path.exists('/home/pi/WXtext.txt'): os.remove('/home/pi/WXtext.txt')
	logfile = open('/home/pi/WXtext.txt', 'a+')
	#logfile.write ('_'+str(int(round(float(winddir[-1])))).rjust(3,'0')+'/'+str(int(round(float(windspeed[-1])))).rjust(3,'0')+'g'+str(int(round(float(windgust[-1])))).rjust(3,'0') +'t'+ str(int(round(float(outsidetemp[-1])))).rjust(3,'0') +'r'+ str(int(round(float(rain[-1])))).rjust(3,'0') +'h'+ str(int(round(float(humidity[-1])))).rjust(2,'0')  +'b'+str(int(round(float(baropressure[-1])/.00295301))).rjust(5,'0'))
	logfile.write (str(int(round(float(winddir[-1])))).rjust(3,'0')+'/'+str(int(round(float(windspeed[-1])))).rjust(3,'0')+'g'+str(int(round(float(windgust[-1])))).rjust(3,'0') +'t'+ str(int(round(float(outsidetemp[-1])))).rjust(3,'0') +'r'+ str(int(round(float(rain[-1])))).rjust(3,'0') +'h'+ str(int(round(float(humidity[-1])))).rjust(2,'0')  +'b'+str(int(round(float(baropressure[-1])/.00295301))).rjust(5,'0'))
	logfile.write('\n')
	logfile.close()
	print 'File written'
else:
	print 'CRC bad - skipped'

#write telemetry beacon file
if os.path.exists('/home/pi/btelemetry.txt'): os.remove('/home/pi/btelemetry.txt')
telemfile = open('/home/pi/btelemetry.txt', 'a+')
telemfile.write ('T#002,'+str(int(round(float(clubhousetemp[-1])))).rjust(3,'0')+','+str(int(round(float(motionsense[-1])))).rjust(3,'0') +','+ str(int(round(float(lightningcount[-1])))).rjust(3,'0')+',000,000,00001111')
telemfile.write('\n')
telemfile.close()

