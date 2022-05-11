#
#
# SDL_Pi_GrovePowerSave
# Raspberry Pi Driver for the SwitchDoc Labs GrovePowerSave 
#
# SwitchDoc Labs
# April  2017
#
# Version 1.1

import RPi.GPIO as GPIO

GrovePowerSave_Default_GPIO_Pin = 20

class SDL_Pi_GrovePowerSave:
	
	def __init__(self, GPIOPin=GrovePowerSave_Default_GPIO_Pin, initialState = True):
                self._GPIOPin = GPIOPin
                self._initialState = initialState
		
		GPIO.setmode(GPIO.BCM)

		GPIO.setup(self._GPIOPin,GPIO.OUT, initial=self._initialState)

	def setPowerSave(self, value):

		GPIO.output(self._GPIOPin, value)


		
		
