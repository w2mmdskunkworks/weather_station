#for graphs
head -1 logfileV2.txt > logcat.txt
tail -8000  logfileV2.txt >> logcat.txt

#for APRS
head -1 logfileV2.txt > APRSlog.txt
tail -2 logfileV2.txt >> APRSlog.txt
