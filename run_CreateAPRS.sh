#!/bin/bash
#echo "Started" >> started.joblog
#whoami >> /home/pi/whoami.joblog
/home/pi/truncate_logfile.sh >> /home/pi/truncate_logfile.joblog 
/usr/bin/python2 /home/pi/code/CreateAPRS_File.py  >> /home/pi/CreateAPRSfile.joblog
