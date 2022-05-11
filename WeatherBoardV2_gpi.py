#!/usr/bin/env python
#
# Weather Board Test File
# Version 1.8 August 22, 2016
#
# SwitchDoc Labs
# www.switchdoc.com
#

# imports

import sys
import time
from datetime import datetime
import random 
import binascii
import struct
import grovepi 

import config

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
import SDL_Pi_WeatherRack as SDL_Pi_WeatherRack
import SDL_DS3231

import SDL_Pi_FRAM
from RPi_AS3935 import RPi_AS3935

import SDL_Pi_INA3221
import SDL_Pi_TCA9545

import SDL_Adafruit_ADS1x15 #ADC controller for air quality
#/*=========================================================================
#    I2C ADDRESS/BITS
#    -----------------------------------------------------------------------*/
TCA9545_ADDRESS =                         (0x73)    # 1110011 (A0+A1=VDD)
#/*=========================================================================*/

#/*=========================================================================
#    CONFIG REGISTER (R/W)
#    -----------------------------------------------------------------------*/
TCA9545_REG_CONFIG            =          (0x00)
#    /*---------------------------------------------------------------------*/

TCA9545_CONFIG_BUS0  =                (0x01)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS1  =                (0x02)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS2  =                (0x04)  # 1 = enable, 0 = disable 
TCA9545_CONFIG_BUS3  =                (0x08)  # 1 = enable, 0 = disable 

#/*=========================================================================*/

import Adafruit_SSD1306
import Scroll_SSD1306
#Initialize air quality sensor

ADS1115 = 0x01	# 16-bit ADC

# Select the gain
# gain = 6144  # +/- 6.144V
gain = 4096  # +/- 4.096V
# gain = 2048  # +/- 2.048V
# gain = 1024  # +/- 1.024V
# gain = 512   # +/- 0.512V
# gain = 256   # +/- 0.256V

# Select the sample rate
# sps = 8    # 8 samples per second
# sps = 16   # 16 samples per second
# sps = 32   # 32 samples per second
# sps = 64   # 64 samples per second
# sps = 128  # 128 samples per second
sps = 250  # 250 samples per second
# sps = 475  # 475 samples per second
# sps = 860  # 860 samples per second

# Initialise the ADC using the default mode (use default I2C address)
adc = SDL_Adafruit_ADS1x15.ADS1x15(ic=ADS1115)

###########################
#Initialize output file
###########################

import os

RewriteLogFileHeaders = False
WriteLogFile = False

if (RewriteLogFileHeaders == True) :
	if os.path.exists('/home/pi/logfileV2.txt'): os.remove('/home/pi/logfileV2.txt')
	#Write headers on new log file
	logfile = open('/home/pi/logfileV2.txt', 'a+')
	logfile.write('Date,Time,WindSpeed,WindDir,WindVoltage,Rain,ClubhouseTemp,OutsideTemp,OutsideHumidity,Visible,IR,UV,LightningMessage,LightningCount,AirQualityMetric,AirQuality,BaroPressure,MotionSense,AM2315crc,WindGust,Temp1,Temp3,Temp4,Temp5')
	logfile.write('\n')
	logfile.close()

###########################
#Sunlight sensor
###########################
import SDL_Pi_SI1145

# Read the sensor
ReadLightSensor = True
print ('Testing here')
#initialize values for log write if sensor is not present
vis=0
IR=0
UV=0
if (ReadLightSensor == True):

	tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = TCA9545_CONFIG_BUS3)
	#Light sensor is on BUS 2 - changed to bus 3 6/26/19
	#tca9545.write_control_register(TCA9545_CONFIG_BUS2)
	sensor = SDL_Pi_SI1145.SDL_Pi_SI1145()
	# read the control register back
	control_register = tca9545.read_control_register()
	vis = sensor.readVisible()
	IR = sensor.readIR()
	UV = sensor.readUV()
	uvIndex = UV / 100.0
	print('SunLight Sensor read at time: %s' % datetime.now())
	print('		Vis:             ' + str(vis))
	print ('		IR:              ' + str(IR))
	print ('		UV Index:        ' + str(uvIndex))
	tca9545.write_control_register(TCA9545_CONFIG_BUS0)
	# read the control register back
	control_register = tca9545.read_control_register()

################
#Code for LED
################

#Light LEDs  = make FALSE when unoccupied
ShowLEDs = False

print ('Testing LEDs')

#GPIO.setmode (GPIO.BCM)
#RedLED = 4
#GPIO.setup(RedLED,GPIO.OUT)
#GPIO.output(RedLED,False)
#GPIO.output(RedLED,True)
#time.sleep(1)
#GPIO.output(RedLED,False)

#GreenLED=6
#GPIO.setup(GreenLED,GPIO.OUT)
#GPIO.output(GreenLED,False)
#GPIO.output(GreenLED,True)
#time.sleep(1)
#GPIO.output(GreenLED,False)

#YellowLED=12
#GPIO.setup(YellowLED,GPIO.OUT)
#GPIO.output(YellowLED,False)
#GPIO.output(YellowLED,True)
#time.sleep(1)
#GPIO.output(YellowLED,False)

#BlueLED=16
#GPIO.setup(BlueLED,GPIO.OUT)
#GPIO.output(BlueLED,False)
#GPIO.output(BlueLED,True)
#time.sleep(1)
#GPIO.output(BlueLED,False)

WhiteLED=19
GPIO.setup(WhiteLED,GPIO.OUT)
GPIO.output(WhiteLED,False)
GPIO.output(WhiteLED,True)
time.sleep(1)
GPIO.output(WhiteLED,False)
print ('Done Testing LEDs')

#Motion sensor setup
MotionSensorPin=5
MotionSensorState = 0

GPIO.setup(MotionSensorPin,GPIO.IN)
#GPIO.setup(MotionSensorPin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
MotionSensorState=GPIO.input(MotionSensorPin)
print ('Motion sensor initial state ',str(MotionSensorState))

################
# Device Present State Variables
###############

#indicate interrupt has happened from as3936

as3935_Interrupt_Happened = False;
# set to true if you are building the Weather Board project with Lightning Sensor
config.Lightning_Mode = True

# set to true if you are building the solar powered version
config.SolarPower_Mode = False;

config.SunAirPlus_Present = False
config.AS3935_Present = False #Lightning sensor
config.DS3231_Present = False #Real-time clock
config.BMP280_Present = True #Barometric pressure sensor
config.FRAM_Present = False
config.HTU21DF_Present = False #Temp and humidity sensor
config.AM2315_Present = False #Temp/Humidity sensor
config.ADS1015_Present = False #AdaFruit ADC
config.ADS1115_Present = False #ADC
config.OLED_Present = False
config.WXLink_Present = False #wireless board

###############
# setup lightning i2c mux
##############

# points to BUS0 initially - That is where the Weather Board is located
if (config.Lightning_Mode == True):
	tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = TCA9545_CONFIG_BUS0)

def returnStatusLine(device, state):

	returnString = device
	if (state == True):
		returnString = returnString + ":   \t\tPresent" 
	else:
		returnString = returnString + ":   \t\tNot Present"
	return returnString

###############   
#WeatherRack Weather Sensors
#
# GPIO Numbering Mode GPIO.BCM
#
anemometerPin = 21
rainPin = 26

# constants
SDL_MODE_INTERNAL_AD = 0
SDL_MODE_I2C_ADS1015 = 1    # internally, the library checks for ADS1115 or ADS1015 if found

#sample mode means return immediately.  THe wind speed is averaged at sampleTime or when you ask, whichever is longer
SDL_MODE_SAMPLE = 0
#Delay mode means to wait for sampleTime and the average after that time.
SDL_MODE_DELAY = 1

weatherStation = SDL_Pi_WeatherRack.SDL_Pi_WeatherRack(anemometerPin, rainPin, 0,0, SDL_MODE_I2C_ADS1015)

weatherStation.setWindMode(SDL_MODE_SAMPLE, 5.0)
#weatherStation.setWindMode(SDL_MODE_DELAY, 5.0)

################

# WXLink Test Setup
WXLink = smbus.SMBus(1)
try:
	data = WXLink.read_i2c_block_data(0x08, 0);
	config.WXLink_Present = True
except:
	config.WXLink_Present = False

################

# DS3231/AT24C32 Setup
filename = time.strftime("%Y-%m-%d%H:%M:%SRTCTest") + ".txt"
starttime = datetime.utcnow()

ds3231 = SDL_DS3231.SDL_DS3231(1, 0x68)

try:
	#comment out the next line after the clock has been initialized
	ds3231.write_now()
	print ("DS3231=\t\t%s" % ds3231.read_datetime())
	config.DS3231_Present = True
	print ("----------------- ")
	print ("----------------- ")
	print (" AT24C32 EEPROM")
	print ("----------------- ")
	print ("writing first 4 addresses with random data")
	for x in range(0,4):
		value = random.randint(0,255)
		print ("address = %i writing value=%i" % (x, value) 	)
		ds3231.write_AT24C32_byte(x, value)
	print ("----------------- ")
	
	print ("reading first 4 addresses")
	for x in range(0,4):
		print ("address = %i value = %i" %(x, ds3231.read_AT24C32_byte(x)) )
	print ("----------------- ")

except IOError as e:
	#    print "I/O error({0}): {1}".format(e.errno, e.strerror)
	config.DS3231_Present = False
	# do the AT24C32 eeprom

################

# BMP280 Setup - internal barometer

try:
	bmp280 = BMP280.BMP280()
	config.BMP280_Present = True

except IOError as e:

	print ("BMP280 I/O error({0}): {1}".format(e.errno, e.strerror))
	config.BMP280_Present = False

################

# HTU21DF Detection 
try:
	HTU21DFOut = subprocess.check_output(["htu21dflib/htu21dflib","-l"])
	config.HTU21DF_Present = True
except:
	config.HTU21DF_Present = False

################

# OLED SSD_1306 Detection
try:
	RST =27
	display = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_address=0x3C)
	# Initialize library.
	display.begin()
	display.clear()
	display.display()
	config.OLED_Present = True
except:
	config.OLED_Present = False

################

# ad3935 Set up Lightning Detector
if (config.Lightning_Mode == True):
	# switch to BUS1 - lightning detector is on Bus1
	tca9545.write_control_register(TCA9545_CONFIG_BUS1)

# ThunderBoard has address 0x02, not 0x03
	as3935 = RPi_AS3935(address=0x02, bus=1)

	try:
		as3935.set_indoors(True)
		config.AS3935_Present = True
		print ("as3935 present")

	except IOError as e:

    		print ("LightningDet I/O error({0}): {1}".format(e.errno, e.strerror))
    		config.AS3935_Present = False
		# back to BUS0
		tca9545.write_control_register(TCA9545_CONFIG_BUS0)

	if (config.AS3935_Present == True):
		i2ccommand = "sudo i2cdetect -y 1"	
		output = subprocess.check_output (i2ccommand,shell=True, stderr=subprocess.STDOUT )
		print output
		as3935.set_noise_floor(0)
		as3935.calibrate(tun_cap=0x0F)

	as3935LastInterrupt = 0
	as3935LightningCount = 0
	as3935LastDistance = 0
	as3935LastStatus = ""
	# back to BUS0
	tca9545.write_control_register(TCA9545_CONFIG_BUS0)

def respond_to_as3935_interrupt():
    # switch to BUS1 - lightning detector is on Bus1
    print ("in respond to as3935 interrupt")
    tca9545.write_control_register(TCA9545_CONFIG_BUS1)
    time.sleep(0.003)
    global as3935, as3935LastInterrupt, as3935LastDistance, as3935LastStatus
    reason = as3935.get_interrupt()
    as3935LastInterrupt = reason
    if reason == 0x01:
	as3935LastStatus = "Noise Floor too low. Adjusting"
        as3935.raise_noise_floor()
    elif reason == 0x04:
	as3935LastStatus = "Disturber detected - masking"
        as3935.set_mask_disturber(True)
    elif reason == 0x08:
        now = datetime.now().strftime('%H:%M:%S - %Y/%m/%d')
        distance = as3935.get_distance()
	as3935LastDistance = distance
	as3935LastStatus = "Lightning Detected "  + str(distance) + "km away. (%s)" % now
    # switch back to BUS0 
    tca9545.write_control_register(TCA9545_CONFIG_BUS0)
    #GPIO.add_event_detect(as3935pin, GPIO.RISING, callback=handle_as3935_interrupt)

if (config.Lightning_Mode == True):
	as3935pin = 13

	if (config.AS3935_Present == True):
		GPIO.setup(as3935pin, GPIO.IN)
		GPIO.add_event_detect(as3935pin, GPIO.RISING)
		#GPIO.add_event_detect(as3935pin, GPIO.RISING, callback=handle_as3935_interrupt)

###############
# Set up FRAM 
fram = SDL_Pi_FRAM.SDL_Pi_FRAM(addr = 0x50)
# FRAM Detection 
try:
	fram.read8(0)
	config.FRAM_Present = True
except:
	config.FRAM_Present = False

###############   
# set up SunAirPlus
if (config.SolarPower_Mode == True):
	
	try:
    		# switch to BUS2 -  SunAirPlus is on Bus2
    		tca9545.write_control_register(TCA9545_CONFIG_BUS2)
    		sunAirPlus = SDL_Pi_INA3221.SDL_Pi_INA3221(addr=0x40)
		# the three channels of the INA3221 named for SunAirPlus Solar Power Controller channels (www.switchdoc.com)
		LIPO_BATTERY_CHANNEL = 1
		SOLAR_CELL_CHANNEL   = 2
		OUTPUT_CHANNEL       = 3

	  	busvoltage1 = sunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL)
		config.SunAirPlus_Present = True
	except:
		config.SunAirPlus_Present = False

    	tca9545.write_control_register(TCA9545_CONFIG_BUS0)

###############
# Detect AM2315  External temp/humidity sensor
try:
	from tentacle_pi.AM2315 import AM2315
	try:
		am2315 = AM2315(0x5c,"/dev/i2c-1")
		temperature, humidity, crc_check = am2315.sense()
		print "AM2315 =", temperature
		config.AM2315_Present = True
		#if (crc_check == -1):
		#	config.AM2315_Present = False
	except:
		config.AM2315_Present = False
except:
	config.AM2315_Present = False
	print ("------> See Readme to install tentacle_pi")

###########
# WXLink functions

def hex2float(s):
    return struct.unpack('<f', binascii.unhexlify(s))[0]

def hex2int(s):
    return struct.unpack('<L', binascii.unhexlify(s))[0]

# Main Loop - sleeps 10 seconds
# Tests all I2C and WeatherRack devices on Weather Board 



###################################################################################
# Main Program

print ""
print "Weather Board Demo / Test Version 1.6 - SwitchDoc Labs"
print ""
print ""
print "Program Started at:"+ time.strftime("%Y-%m-%d %H:%M:%S")
print ""

totalRain = 0

print ("----------------------")
print returnStatusLine("DS3231",config.DS3231_Present)
print returnStatusLine("BMP280",config.BMP280_Present)
print returnStatusLine("FRAM",config.FRAM_Present)
print returnStatusLine("HTU21DF",config.HTU21DF_Present)
print returnStatusLine("AM2315",config.AM2315_Present)
print returnStatusLine("ADS1015",config.ADS1015_Present)
print returnStatusLine("ADS1115",config.ADS1115_Present)
print returnStatusLine("AS3935",config.AS3935_Present)
print returnStatusLine("OLED",config.OLED_Present)
print returnStatusLine("SunAirPlus",config.SunAirPlus_Present)
print returnStatusLine("WXLink",config.WXLink_Present)
print "----------------------"

block1 = ""
block2 = ""

while True:
	
	#Reset air quality LEDs
	#GPIO.output(GreenLED,False)
	#GPIO.output(YellowLED,False)
	#GPIO.output(RedLED,False)
	#GPIO.output(BlueLED,False)
	#Test air quality
	rawCh0 = adc.readRaw(0, gain, sps) 
	#print ("Air Quality =%d" % (rawCh0))
	if 0 < rawCh0 < 3199: 
		Airquality = 'Fresh Air'
	#	GPIO.output(GreenLED,ShowLEDs)	
	if 3200 < rawCh0 < 4799: 
		Airquality = 'Low Pollution'
	#	GPIO.output(BlueLED,ShowLEDs)
	if 4800 < rawCh0 < 6399: 
		Airquality = 'Medium Pollution'
	#	GPIO.output(BlueLED,ShowLEDs)
	if 6400 < rawCh0 < 11199: 
		Airquality = 'High Pollution'
	#	GPIO.output(RedLED,ShowLEDs)
	if 12000 < rawCh0 < 65536: 
		Airquality = 'Very High Pollution'
	#	GPIO.output(GreenLED,False)
	#	GPIO.output(RedLED,ShowLEDs)
	#print (rawCh0,Airquality)
	
	if (config.Lightning_Mode == True):
    		# switch to BUS0
		print "switch to Bus0"
    		tca9545.write_control_register(TCA9545_CONFIG_BUS0)

	print "---------------------------------------- "
	print "----------------- "
	if (config.DS3231_Present == True):
		print " DS3231 Real Time Clock"
	else:
		print " DS3231 Real Time Clock Not Present"
	
	print "----------------- "
	#

	if (config.DS3231_Present == True):
		currenttime = datetime.utcnow()

		deltatime = currenttime - starttime
	 
		print "Raspberry Pi=\t" + time.strftime("%Y-%m-%d %H:%M:%S")

		if (config.OLED_Present):
			Scroll_SSD1306.addLineOLED(display,"%s" % ds3231.read_datetime())

		print "DS3231=\t\t%s" % ds3231.read_datetime()
	
		print "DS3231 Temperature= \t%0.2f C" % ds3231.getTemp()
		print "----------------- "

	print "----------------- "
	print " WeatherRack Weather Sensors" 
 
 	if (config.WXLink_Present == True):
		print " WXLink Remote WeatherRack"
	else:
		print " WeatherRack Local"	
	print "----------------- "
	#
	print "----------------- "

	if (config.AM2315_Present == True):
		print " AM2315 Temperature/Humidity Sensor"
	else:
		print " AM2315 Temperature/Humidity  Sensor Not Present"
	print "----------------- "

	if (config.AM2315_Present):
    		temperature, humidity, crc_check = am2315.sense()
    		print "AM2315 temperature: %0.1f" % (temperature *.94 * 9 / 5 + 32) #Sensor correction
    		print "AM2315 humidity: %0.1f" % humidity
    		print "AM2315 crc: %s" % crc_check
	print "----------------- "
	print "----------------- "


#If the remote link isn't installed (which is isn't at W2MMD) read the values from the cables
	if (config.WXLink_Present == False):
 		currentWindSpeed = weatherStation.current_wind_speed()/1.6
  		currentWindGust = weatherStation.get_wind_gust()/1.6
  		totalRain = totalRain + weatherStation.get_current_rain_total()/25.4
		#this causes error
		currentWindDirection = weatherStation.current_wind_direction()
		print ("Wind Direction=\t%0.2f")%(currentWindDirection)
  		print("Rain Total=\t%0.2f in")%(totalRain)
  		print("Wind Speed=\t%0.2f MPH")%(currentWindSpeed)
		print("MPH wind_gust=\t%0.2f MPH")%(currentWindGust)
	
	#Print light sensor results
	if (ReadLightSensor == True):
		tca9545 = SDL_Pi_TCA9545.SDL_Pi_TCA9545(addr=TCA9545_ADDRESS, bus_enable = TCA9545_CONFIG_BUS3)
		#tca9545.write_control_register(TCA9545_CONFIG_BUS2)
		sensor = SDL_Pi_SI1145.SDL_Pi_SI1145()
		# read the control register back
		control_register = tca9545.read_control_register()
		vis = sensor.readVisible()
		IR = sensor.readIR()
		UV = sensor.readUV()
		uvIndex = UV 
		print "----------------- "
		print('SunLight Sensor read at time: %s' % datetime.now())
		print '		Vis:             ' + str(vis)
		print '		IR:              ' + str(IR)
		print '		UV Index:        ' + str(uvIndex)
		print "----------------- "
		tca9545.write_control_register(TCA9545_CONFIG_BUS0)
		# read the control register back
		control_register = tca9545.read_control_register()
	
	if (config.OLED_Present):
		Scroll_SSD1306.addLineOLED(display,  ("Wind Speed=\t%0.2f MPH")%(currentWindSpeed))
		Scroll_SSD1306.addLineOLED(display,  ("Rain Total=\t%0.2f in")%(totalRain))
		if (config.ADS1015_Present or config.ADS1115_Present):	
			Scroll_SSD1306.addLineOLED(display,  "Wind Dir=%0.2f Degrees" % weatherStation.current_wind_direction())

	if (config.ADS1015_Present or config.ADS1115_Present):	
		print "Wind Direction=\t\t\t %0.2f Degrees" % weatherStation.current_wind_direction()
		print "Wind Direction Voltage=\t\t %0.3f V" % weatherStation.current_wind_direction_voltage()

# If the remote link package is present
	if (config.WXLink_Present == True):
		oldblock1 = block1
		oldblock2 = block2	
		try:
   			print "-----------"
   			print "block 1"
   			block1 = WXLink.read_i2c_block_data(0x08, 0);
   			print ''.join('{:02x}'.format(x) for x in block1) 
			block1 = bytearray(block1)
   			print "block 2"
   			block2 = WXLink.read_i2c_block_data(0x08, 1);
   			block2 = bytearray(block2)
			print ''.join('{:02x}'.format(x) for x in block2) 
   			print "-----------"
		except:
			print "WXLink Read failed - Old Data Kept"
			block1 = oldblock1
			block2 = oldblock2

               	currentWindSpeed = struct.unpack('f', str(block1[9:13]))[0]    /1.6
		
               	currentWindGust = 0.0   # not implemented in Solar WXLink version

               	totalRain = struct.unpack('l', str(block1[17:21]))[0]/25.4

               	print("Rain Total 553=\t%0.2f in")%(totalRain)
               	print("Wind Speed 554=\t%0.2f MPH")%(currentWindSpeed)
               	if (config.OLED_Present):
                       	Scroll_SSD1306.addLineOLED(display,  ("Wind Speed=\t%0.2f MPH")%(currentWindSpeed))
                       	Scroll_SSD1306.addLineOLED(display,  ("Rain Total=\t%0.2f in")%(totalRain))
                       	Scroll_SSD1306.addLineOLED(display,  "Wind Dir=%0.2f Degrees" % weatherStation.current_wind_direction())

               	currentWindDirection = struct.unpack('H', str(block1[7:9]))[0] 
                print "Wind Direction 564=\t\t\t %i Degrees" % currentWindDirection

		# now do the AM2315 Temperature
               	temperature = struct.unpack('f', str(block1[25:29]))[0] 
                elements = [block1[29], block1[30], block1[31], block2[0]]
        	outHByte = bytearray(elements)
        	humidity = struct.unpack('f', str(outHByte))[0]
    		print "AM2315 from WXLink temperature: %0.1f" % (temperature * 9 / 5 + 32)
    		print "AM2315 from WXLink humidity: %0.1f" % humidity

		# now read the SunAirPlus Data from WXLink

		batteryVoltage = struct.unpack('f', str(block2[1:5]))[0]
        	batteryCurrent = struct.unpack('f', str(block2[5:9]))[0]
        	loadCurrent = struct.unpack('f', str(block2[9:13]))[0]
        	solarPanelVoltage = struct.unpack('f', str(block2[13:17]))[0]
        	solarPanelCurrent = struct.unpack('f', str(block2[17:21]))[0]

        	auxA = struct.unpack('f', str(block2[21:25]))[0]


        	print "WXLink batteryVoltage = %6.2f" % batteryVoltage
        	print "WXLink batteryCurrent = %6.2f" % batteryCurrent
        	print "WXLink loadCurrent = %6.2f" % loadCurrent
        	print "WXLink solarPanelVoltage = %6.2f" % solarPanelVoltage
        	print "WXLink solarPanelCurrent = %6.2f" % solarPanelCurrent
        	print "WXLink auxA = %6.2f" % auxA

		# message ID
		MessageID = struct.unpack('l', str(block2[25:29]))[0]
    		print "WXLink Message ID %i" % MessageID
		
#Getting failures here so test for BMP280 again
# BMP280 Setup - internal barometer

	try:
		bmp280 = BMP280.BMP280()
		config.BMP280_Present = True

	except IOError as e:

		print "BMP280 I/O error({0}): {1}".format(e.errno, e.strerror)
		config.BMP280_Present = False

#Read from BMP280
	print "----------------- "
	print "----------------- "
	if (config.BMP280_Present == True):
		print " BMP280 Barometer"
	else:
		print " BMP280 Barometer Not Present"
	print "----------------- "

	if (config.BMP280_Present):
		
		print 'Temperature = \t{0:0.2f} F'.format(bmp280.read_temperature()*9/5+32)
		print 'Pressure = \t{0:0.2f} KPa'.format((bmp280.read_pressure()/1000)*0.2953)
		#Causes error
		#print 'Altitude = \t{0:0.2f} m'.format(bmp280.read_altitude())
		print 'Sealevel Pressure = \t{0:0.2f} KPa'.format((bmp280.read_sealevel_pressure()/1000)*0.2953)
		if (config.OLED_Present):
			Scroll_SSD1306.addLineOLED(display, 'Press= \t{0:0.2f} KPa'.format(bmp280.read_pressure()/1000))
			if (config.HTU21DF_Present == False):
				Scroll_SSD1306.addLineOLED(display, 'InTemp= \t{0:0.2f} F'.format(bmp280.read_temperature()*(9/5)+32))
	print "----------------- "

	print "----------------- "
	if (config.HTU21DF_Present == True):
		print " HTU21DF Temp/Hum"
	else:
		print " HTU21DF Temp/Hum Not Present"
	print "----------------- "

	# We use a C library for this device as it just doesn't play well with Python and smbus/I2C libraries
	if (config.HTU21DF_Present):
		HTU21DFOut = subprocess.check_output(["htu21dflib/htu21dflib","-l"])
		splitstring = HTU21DFOut.split()

		HTUtemperature = float(splitstring[0])	
		HTUhumidity = float(splitstring[1])	
		print "Temperature = \t%0.2f C" % HTUtemperature
		print "Humidity = \t%0.2f %%" % HTUhumidity
		if (config.OLED_Present):
			Scroll_SSD1306.addLineOLED(display,  "InTemp = \t%0.2f C" % HTUtemperature)
	print "----------------- "

	print "----------------- "
	if (config.AS3935_Present):
		print " AS3935 Lightning Detector"
	else:
		print " AS3935 Lightning Detector Not Present"
	print "----------------- "

	if (config.AS3935_Present):
		if (GPIO.event_detected(as3935pin)):
			respond_to_as3935_interrupt()

		print "Last result from AS3935:"

		if (as3935LastInterrupt == 0x00):
			LightningMessage = "----No Lightning detected---"
			print LightningMessage
		
		if (as3935LastInterrupt == 0x01):
			LightningMessage = "Noise Floor: %s" % as3935LastStatus
			print LightningMessage
			as3935LastInterrupt = 0x00

		if (as3935LastInterrupt == 0x04):
			LightningMessage = "Disturber: %s" % as3935LastStatus
			print LightningMessage
			as3935LastInterrupt = 0x00

		if (as3935LastInterrupt == 0x08):
			LightningMessage = "Lightning: %s" % as3935LastStatus
			print LightningMessage
			as3935LightningCount += 1
			if (config.OLED_Present):
				Scroll_SSD1306.addLineOLED(display, '')
				Scroll_SSD1306.addLineOLED(display, '---LIGHTNING---')
				Scroll_SSD1306.addLineOLED(display, '')
			as3935LastInterrupt = 0x00

		print "Lightning Count = ", as3935LightningCount
	print "----------------- "
	
	print "----------------- "
	if (config.FRAM_Present):
		print " FRAM Present"
	else:
		print " FRAM Not Present"
	print "----------------- "

        if (config.FRAM_Present):
		print "writing first 3 addresses with random data"
		for x in range(0,3):
			value = random.randint(0,255)
                	print "address = %i writing value=%i" % (x, value)
                	fram.write8(x, value)
        	print "----------------- "

        	print "reading first 3 addresses"
        	for x in range(0,3):
                	print "address = %i value = %i" %(x, fram.read8(x))
        print "----------------- "
	print
	print "----------------- "

	if (config.SunAirPlus_Present):
		print " SunAirPlus Present"
	else:
		print " SunAirPlus Not Present"
	print "----------------- "

	if (config.SolarPower_Mode):
		if (config.SunAirPlus_Present):
    			tca9545.write_control_register(TCA9545_CONFIG_BUS2)
		      	shuntvoltage1 = 0
        		busvoltage1   = 0
        		current_mA1   = 0
        		loadvoltage1  = 0


        		busvoltage1 = sunAirPlus.getBusVoltage_V(LIPO_BATTERY_CHANNEL)
        		shuntvoltage1 = sunAirPlus.getShuntVoltage_mV(LIPO_BATTERY_CHANNEL)
        		# minus is to get the "sense" right.   - means the battery is charging, + that it is discharging
        		current_mA1 = sunAirPlus.getCurrent_mA(LIPO_BATTERY_CHANNEL)

        		loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000)
		
        		print "LIPO_Battery Bus Voltage: %3.2f V " % busvoltage1
        		print "LIPO_Battery Shunt Voltage: %3.2f mV " % shuntvoltage1
        		print "LIPO_Battery Load Voltage:  %3.2f V" % loadvoltage1
        		print "LIPO_Battery Current 1:  %3.2f mA" % current_mA1
        		print

        		shuntvoltage2 = 0
        		busvoltage2 = 0
        		current_mA2 = 0
        		loadvoltage2 = 0

        		busvoltage2 = sunAirPlus.getBusVoltage_V(SOLAR_CELL_CHANNEL)
        		shuntvoltage2 = sunAirPlus.getShuntVoltage_mV(SOLAR_CELL_CHANNEL)
        		current_mA2 = -sunAirPlus.getCurrent_mA(SOLAR_CELL_CHANNEL)
        		loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000)

        		print "Solar Cell Bus Voltage 2:  %3.2f V " % busvoltage2
        		print "Solar Cell Shunt Voltage 2: %3.2f mV " % shuntvoltage2
        		print "Solar Cell Load Voltage 2:  %3.2f V" % loadvoltage2
        		print "Solar Cell Current 2:  %3.2f mA" % current_mA2
        		print

        		shuntvoltage3 = 0
        		busvoltage3 = 0
        		current_mA3 = 0
        		loadvoltage3 = 0

        		busvoltage3 = sunAirPlus.getBusVoltage_V(OUTPUT_CHANNEL)
        		shuntvoltage3 = sunAirPlus.getShuntVoltage_mV(OUTPUT_CHANNEL)
        		current_mA3 = sunAirPlus.getCurrent_mA(OUTPUT_CHANNEL)
        		loadvoltage3 = busvoltage3 + (shuntvoltage3 / 1000)

        		print "Output Bus Voltage 3:  %3.2f V " % busvoltage3
        		print "Output Shunt Voltage 3: %3.2f mV " % shuntvoltage3
        		print "Output Load Voltage 3:  %3.2f V" % loadvoltage3
        		print "Output Current 3:  %3.2f mA" % current_mA3
        		print
    			tca9545.write_control_register(TCA9545_CONFIG_BUS1)
	
	MotionSensorState=GPIO.input(MotionSensorPin)
	print ('Motion sensor = ',str(MotionSensorState))
	if MotionSensorState==1:
		GPIO.output(WhiteLED,True)
	else: 
		GPIO.output(WhiteLED,False)
	
	#Write the log file
	if WriteLogFile == True:	
		logfile = open('/home/pi/logfileV2.txt', 'a+')
		#logfile.write("DS3231=\t\t%s" % ds3231.read_datetime())
		logfile.write(str(ds3231.read_datetime())[0:10]) #Date
		logfile.write (',')
		logfile.write(str(ds3231.read_datetime())[11:]) #Time
		logfile.write (',')
		#logfile.write(totalRain)
		#logfile.write('\n')
		logfile.write(str(currentWindSpeed))
		#logfile.write(("Wind Speed=\t%0.2f MPH")%(currentWindSpeed))
		logfile.write (',')
		#Replace wind direction with BP
		logfile.write(str(weatherStation.current_wind_direction()))

		logfile.write (',')
		logfile.write(str(weatherStation.current_wind_direction_voltage()))
		logfile.write (',')
		logfile.write(str(totalRain))
		logfile.write (',')
		logfile.write(str(ds3231.getTemp()*9/5+32))
		logfile.write(',')
		logfile.write(str(temperature*9/5+32))
		logfile.write(',')
		logfile.write(str(humidity))
		#add light sensor values to log
		logfile.write(',')
		logfile.write(str(vis))
		logfile.write(',')
		logfile.write(str(IR))
		logfile.write(',')
		logfile.write(str(UV))
		logfile.write(',')
		logfile.write (LightningMessage)
		logfile.write(',')
		logfile.write(str(as3935LightningCount))
		logfile.write(',')
		logfile.write(str(rawCh0)) #air quality metric
		logfile.write (',')
		logfile.write(Airquality)
		logfile.write (',')			
		logfile.write(str((bmp280.read_pressure()/1000)*0.2953))
		logfile.write (',')	
		logfile.write(str(GPIO.input(MotionSensorPin)))
		logfile.write (',')	
		logfile.write(str(crc_check))
		logfile.write (',')
		logfile.write(str(currentWindGust))
		logfile.write(',0,0,0,0')
		logfile.write('\n')
		logfile.close()
		
				
	print "Sleeping 60 seconds for V2"
	print ""
	print ""
	time.sleep(10.0)

