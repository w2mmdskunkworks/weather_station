
print ('Starting')
import sys
import time
from datetime import datetime
import random 
import binascii
import struct



import subprocess
import RPi.GPIO as GPIO
import smbus
#

sys.path.append('./SDL_Pi_SSD1306')
sys.path.append('./SDL_Pi_INA3221')
sys.path.append('./RTC_SDL_DS3231')
sys.path.append('./Adafruit_Python_BMP')
sys.path.append('./Adafruit_Python_GPIO')
sys.path.append('./Adafruit_Python_SSD1306')
sys.path.append('./SDL_Pi_WeatherRack')
sys.path.append('./SDL_Pi_FRAM')
sys.path.append('./SDL_Pi_TCA9545')
sys.path.append('./RaspberryPi-AS3935/RPi_AS3935')
sys.path.append('./SDL_Pi_SI1145'); # Sun sensor

import Adafruit_BMP.BMP280 as BMP280
bmp280 = BMP280.BMP280()

while True:		
	print 'Temperature = \t{0:0.2f} F'.format(bmp280.read_temperature()*9/5+32)
	print 'Pressure = \t{0:0.2f} KPa'.format(bmp280.read_pressure()/1000)
	time.sleep(10.0)
