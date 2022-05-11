#!/bin/bash
head -1000 logcat.txt >  logcat_for_influx.txt
sed -i 's/,/ /' logcat_for_influx.txt
sed -i 's/Date Time/DateTime/' logcat_for_influx.txt
sed -i 's/:[0-9][0-9],/:00,/' logcat_for_influx.txt

export_csv_to_influx --csv /home/pi/logcat_for_influx.txt\
 --dbname weather5\
 --measurement Weather\
 --delimiter ','\
 --user admin --password JonWb2mnf\
 --time_column DateTime\
# --field_columns "*"\
 --field_columns WindSpeed,WindDir,WindVoltage,Rain,ClubhouseTemp,\
OutsideTemp,OutsideHumidity,Visible,IR,UV,LightningMessage,LightningCount\
,AirQualityMetric,AirQuality,BaroPressure,MotionSense,AM2315crc,WindGust,Temp1,Temp3,Temp4,Temp5\
 --server 127.0.0.1:8086\
 --force_insert_even_csv_no_update\
 --drop_database\
 --drop_measurement
	

