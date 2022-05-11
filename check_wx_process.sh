#!/bin/bash
#Checks for  running status of last WX process
#If that process doesn't exist reboot
dt=`date '+%d/%m/%Y %H:%M:%S'`
#echo "$dt"

LAST_PROCESS=$(tail /home/pi/weather-station/WeatherBoard/WX_process_log.txt)
echo $LAST_PROCESS
ps -h  $LAST_PROCESS
RETURN_CODE=$?
echo $?
echo $RETURN_CODE
if [ "$RETURN_CODE" -eq "1" ]
then
	printf "Rebooting $dt \n\r" #>> /home/pi/weather-station/WeatherBoard/WXreboot.log
	cp /home/pi/weather-station/WeatherBoard/WX_linelog.log /home/pi/CrashLog.log
	logger "WX_reboot"
	sleep 1
	sudo reboot
else
	printf "Continuing $dt \n\r" #>>  /home/pi/weather-station/WeatherBoard/WXreboot.log
fi
 
