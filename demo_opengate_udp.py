#!/usr/bin/env python
# demo_opengate_udp.py
######################
import time
import traceback
import serial
import SDL_Pi_HDC1000
import Adafruit_ADS1x15
import RPi.GPIO as gpio

import binascii
import json
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)


###################################### 

OG_UDP_HOST  = '35.157.105.177'
OG_UDP_PORT = '10000'
OG_API_KEY = 'bc7b7767-6430-4b3c-b4aa-f1b16cd66d4a'
DEVICE_ID = 'sixfab01'
SERIAL_PORT = '/dev/ttyS0'
SERIAL_BAUDRATE=9600
SAMPLING_PERIOD = 300

###################################### 
'''
#Relay
relay = 26
print'RELAY TEST'
gpio.setup(relay,gpio.OUT)
gpio.output(relay,gpio.HIGH)
time.sleep(1)
gpio.output(relay,gpio.LOW)
time.sleep(0.5)
'''
#USER LED
led = 20 
print'LED TEST'
gpio.setup(led,gpio.OUT)
gpio.output(led,gpio.HIGH)
time.sleep(1)
gpio.output(led,gpio.LOW)
time.sleep(0.5)

#ADS1X15

adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)
GAIN = 1

#print('Channel 0: {0}'.format(adc.read_adc(2, gain=GAIN)))
print 'ALL CHANNEL'
for i in range(4):
        print adc.read_adc(i, gain=GAIN)

#Lux
LUXCHANNEL = 0
rawLux = adc.read_adc(LUXCHANNEL,gain=GAIN)
lux = (rawLux * 100)/1580
print 'LUX: %d' %lux

#Tempertaure and Humidity
hdc = SDL_Pi_HDC1000.SDL_Pi_HDC1000()
temperature = hdc.readTemperature()
humidity = hdc.readHumidity()
print 'Temperature: %d' %temperature
print 'Humidity: %d' %humidity

ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=SERIAL_BAUDRATE,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

#### NEW
def ser_write_sleep(at_command, sleep_seconds=1):
    ser.write(at_command+'\r')
    time.sleep(sleep_seconds)

def ser_write_until_ok(at_command, sleep_seconds=1):
    ser_write_sleep(at_command,sleep_seconds)
    while True:
        response = ser.readline()
        print 'RESPONSE:%s' % response  
        if response.startswith('OK') or response.startswith('ERROR'):
            break
    return response

'''
def ser_write_until_ok(at_command, sleep_seconds=1):
    while True:
        response = ser_write(at_command, sleep_seconds)
        if response.startswith('OK') or response.startswith('ERROR'):
            break
'''

##########

def enable_wan_comms():
    print 'Enabling WAN comms'
    while True:

        ser.reset_input_buffer()
        ser_write_until_ok('ATE1')

        print 'DONE 0'
        ser.reset_input_buffer()
        ser_write_until_ok('AT+NRB')    

        time.sleep(2)

        ser.reset_input_buffer()
        ser_write_until_ok('AT+CFUN=1')    

        print 'DONE 1'
        ser.reset_input_buffer()
        ser_write_until_ok('AT+CGATT=1',0.5)

#        print 'DONE 2'
#        ser.reset_input_buffer()
#        ser_write_until_ok('AT+CGDCONT=1,"IP","IOTTEST"')

        print 'DONE 2'
        ser.reset_input_buffer()
        ser_write_until_ok('AT+NBAND=20')

        ser.reset_input_buffer()
        ser_write_until_ok('AT+COPS=1,2,"21401"')

        ser.reset_input_buffer()
        ser_write_until_ok('AT+NUESTATS')

        print 'DONE 3'
        time.sleep(3)
        ser.reset_input_buffer()
        while True: 
            ser_write_sleep('AT+CGATT?')
            while 1:
                ser_write_sleep('AT+CGATT?\r')
                response = ser.readline()
                response = ser.readline()
                print 'RESPONSE:%s' % response
                if response.startswith('+CGATT:1'):
                    break
            break

        print 'DONE 4'
        ser_write_sleep('AT+NSOCL=0')

        ser.reset_input_buffer()
        while True:
            ser_write_sleep('AT+NSOCR=DGRAM,17,3005,1',0.5)
            while True:
                response = ser.readline()
                response = ser.readline()
                print 'RESPONSE:%s' % response
                    
                if response.startswith('OK'):
                    break
                                    
                if response.startswith('ERROR'):
                    ser.write('AT+NSOCR=DGRAM,17,3005,1\r')
                            
            break

        print 'DONE 5'
            
        break


def current_millis():
    '''Gets current millis'''
    return long(round(time.time()))


def get_one_datapoint(datastream, value):
    '''Prepare data points to send'''
    datapoints = {
        'version': '1.0.0',
        'device': DEVICE_ID,
        'apikey': OG_API_KEY,
        'datastreams': [
            {
                'id': datastream,
                'datapoints': [
                    {
                    #    'at': current_millis(),
                        'value': value
                    }
                 ]
            }
        ]
    }
    return datapoints


def publish_data(datapoints):

    print datapoints
    encoded_datapoints = datapoints.encode('hex')
    data_legth = str(len(datapoints))
    format_data = (OG_UDP_HOST, OG_UDP_PORT, data_legth, encoded_datapoints)
    data = 'AT+NSOST=0,{0},{1},{2},{3}\r'.format(*format_data)
    print data
    ser.reset_input_buffer()
    ser_write_until_ok(data, 2)
#    ser.write(data)
#    response = ser.readline()
#    response = ser.readline()
#    print 'RESPONSE:%s' % response


def sample_and_send():
    temp = round(hdc.readTemperature(), 2)
    hum = round(hdc.readHumidity(), 2)
    rawLux = adc.read_adc(0,gain=GAIN)
    lux = round((adc.read_adc(LUXCHANNEL,gain=GAIN) * 100)/1580, 2)
    
    values = (('temperature', temp), ('humidity', hum), ('lux', lux))
    print values
    
    for value in values:
        datapoints_dict = get_one_datapoint(*value)
        datapoints_str = json.dumps(datapoints_dict) 
        publish_data(datapoints_str)


def main():
    try:
        enable_wan_comms()
        while True:
            sample_and_send()
            time.sleep(SAMPLING_PERIOD)
    except KeyboardInterrupt:
        print 'Bye'
    except:
        traceback.print_exc()



if __name__ == '__main__':
    main()
