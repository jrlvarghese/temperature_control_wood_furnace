#Wood furnace temperature control with a shutter
Uses a DS18B20 temperature sensor to sense temperature
AH20 temperature and humidity sensor will be added in future
2 TM1637 display are used one for displaying set temperature and menu management and other for displaying current readings

Pin 8,9,10,11 is used for displays
Pin 2,3,4 is used for Rotary encoder
Pin 5 for DS18B20 temperature sensor
Pin 13 for shutter actuator (logic output state could be managed using menu)
Pin A4, A5 is for I2C communication with external eeprom and AHT sensor in future

Menu have 3 parameters to set:
    P1 - Set temperature
    P2 - Temperature hysteresis
    P3 - Output state of shutter actuator