#!/bin/bash
cd /home/pi/weather-station/WeatherBoard
python WeatherBoardV6.py > WX_linelog.log  & echo $! > WX_process_log.txt  &

