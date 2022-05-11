import AM2315 
sens = AM2315.AM2315()
print "Passed initiation"
print sens.read_temperature()
print sens.read_humidity()
print sens.read_humidity_temperature()
print sens.read_humidity_temperature_crc()
