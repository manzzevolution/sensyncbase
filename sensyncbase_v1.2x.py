#!/usr/bin/python
 
#V0 : basic measurement, averaging, saving
#V1.1: exception serial CO2 & PM preventing USB-sensor fault, heater control
#V1.2: LCD integration, sensor unit

'''
-------------------- BASE DEPOK ----------------------
'''

import serial
import datetime
import os
import lcddriver
from time import *
import time, signal, sys, RPi.GPIO as GPIO
from Adafruit_ADS1x15 import ADS1x15
from Sht1x import Sht1x as SHT1x
from datetime import datetime
import urllib
import socket

socket.setdefaulttimeout(5)
 
id='xx-bplhd-jabar-dpk'
url_upload='http://secure.getsensync.com/proc/bplhd_jabar/base/process.php'
url_start='http://secure.getsensync.com/proc/bplhd_jabar/base/start.php?id='
lcd = lcddriver.lcd()

sys.path.append('./Adafruit_ADS1x15')
 
import SDL_Pi_Weather_80422 as SDL_Pi_Weather_80422
 
GPIO.setwarnings(False)
 
sht1x=SHT1x(18,7)     #GPIO Board
#sht1x =SHT1x(24,4)      #GPIO BCM
pump_pin = 16 #19           # switch 5 in cabling
valve_a_pin = 31 #12        # Valve A - Zeroing Filter
valve_b_pin = 32 #6         # Valve B - Ambient Air
power_pm_pin = 16 #23       # Charger Control
anenometerPin = 8 #17
rainPin = 12 #18
heater_pin = 36 #5
 
Nv=""
Tv=""
Vv=""
response=""
response1=""
signal_val=""
counter=0
CO2_val=0.0
co2_status=1
pm_status=1
port1check=1
port2check=1
heater_threshold=50.0
 
#Alphasense Gas Sensor Constants [WE,WAux,Sens]
#punya bandung
# c_sensor1 = [331,329,236.0] #O3
# c_sensor2 = [440,311,403.0] #CO
# c_sensor3 = [216,217,196.0] #NO2
# c_sensor4 = [362,368,326.0] #SO2
# c_sensor5 = [371,356,618.0] #NO
# c_sensor6 = [369,318,316.0]
#punya depok
# c_sensor1 = [326,329,280.0] #O3
# c_sensor2 = [466,300,354.0] #CO
# c_sensor3 = [214,229,170.0] #NO2
# c_sensor4 = [375,356,311.0] #SO2
# c_sensor5 = [380,380,618.0] #NO
# c_sensor6 = [369,318,316.0]
# depok modified, oktober 2017
c_sensor1 = [323,329,260.0] #O3
c_sensor2 = [560,300,554.0] #CO
c_sensor3 = [210,228,150.0] #NO2
c_sensor4 = [364,350,291.0] #SO2
c_sensor5 = [330,400,598.0] #NO
c_sensor6 = [369,318,316.0] #voc blm dimodif, cek dl ambang batasnya
 
param=[20,1.0] #sampling number, interval
sensor_val=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] #0-16 data variable

sensor_name=["O3","CO","NO2","SO2","NO","CO2","VOC","PM1",
             "PM2.5","PM4","PM10","TSP","ERR","TEMP","RH","WS","WD"]
sensor_unit=["ug","ug","ug","ug","ug","ppm","ppm","ug",
             "ug","ug","ug","ug"," ","C","%","kph"," "]

adc=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0, #adc0-15
     0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
#GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BOARD)
 
GPIO.setup(pump_pin, GPIO.OUT)
GPIO.setup(valve_a_pin, GPIO.OUT)
GPIO.setup(valve_b_pin, GPIO.OUT)
GPIO.setup(power_pm_pin, GPIO.OUT)
GPIO.setup(heater_pin, GPIO.OUT)
 
# constants
SDL_MODE_INTERNAL_AD = 0
SDL_MODE_I2C_ADS1015 = 1

#sample mode means return immediately.  THe wind speed is averaged at sampleTime or when you ask, whichever is longer
SDL_MODE_SAMPLE = 0
#Delay mode means to wait for sampleTime and the average after that time.
SDL_MODE_DELAY = 1
 
# weatherStation = SDL_Pi_Weather_80422.SDL_Pi_Weather_80422(anenometerPin, rainPin, 0,0, SDL_MODE_I2C_ADS1015)
 
#weatherStation.setWindMode(SDL_MODE_SAMPLE, 5.0)
# weatherStation.setWindMode(SDL_MODE_DELAY, 20.0)
totalRain = 0

ws_sample_period = 30
ws_counter = 0

def serviceInterruptAnem(self):
	global ws_counter
	# print("gpio %s: %s" % (gpio_id, val))
	# print "falling detected"
	ws_counter += 1
	#print ws_counter

GPIO.setup(anenometerPin, GPIO.IN)
GPIO.add_event_detect(anenometerPin, GPIO.FALLING, callback=serviceInterruptAnem, bouncetime=10)
#GPIO.add_event_detect(anenometerPin, GPIO.FALLING, callback=serviceInterruptAnem)
#GPIO.add_event_detect(anenometerPin, GPIO.FALLING)

 
ADS1015 = 0x00  # 12-bit ADC
ADS1115 = 0x01  # 16-bit ADC
 
# Select the gain
gain = 6144  # +/- 6.144V
#gain = 4096  # +/- 4.096V
# gain = 2048  # +/- 2.048V
# gain = 1024  # +/- 1.024V
# gain = 512   # +/- 0.512V
# gain = 256   # +/- 0.256V
 
# Select the sample rate
sps = 8    # 8 samples per second
#sps = 16   # 16 samples per second
# sps = 32   # 32 samples per second
# sps = 64   # 64 samples per second
# sps = 128  # 128 samples per second
# sps = 250  # 250 samples per second
#sps = 475  # 475 samples per second
# sps = 860  # 860 samples per second
 
# Initialise the ADC using the default mode (use default I2C address)
# Set this to ADS1015 or ADS1115 depending on the ADC you are using!
# adc = ADS1x15(ic=ADS1115)
adc1 = ADS1x15(ic=ADS1115, address=0x48)
adc2 = ADS1x15(ic=ADS1115, address=0x49)
adc3 = ADS1x15(ic=ADS1115, address=0x4A)
adc4 = ADS1x15(ic=ADS1115, address=0x4B)
 
i=0
# port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=3.0) #GPRS Serial TxRx

#CO2
try:
        port1 = serial.Serial("/dev/CO2", baudrate=19200, timeout=3.0) #CO2 Sensor
        port1check=port1.isOpen()
except serial.serialutil.SerialException, e:
        z=e
        co2_status=0
        print(z)

try: #PM Sensor
        port2 = serial.Serial(
                "/dev/PM",
                baudrate=38400,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                writeTimeout =0,
                timeout = 10,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False)
        port2check=port2.isOpen()         
except serial.serialutil.SerialException, e:
        z=e
        pm_status=0
        print(z)
 
if port1check== 1:
        print("Port 1 opened...")
else:
        print("Port 1 failed...")
 
if port2check== 1:
        print("Port 2 opened...")
else:
        print("Port 2 failed...")
 
def signal_handler(signal, frame):
        print 'You pressed Ctrl+C!'
        sys.exit(0)

def getWs():
	global ws_counter
	ws_counter = 0
	#if GPIO.event_detected(anenometerPin):
	time.sleep(ws_sample_period)
	print "counter: " + str(ws_counter)
        ws = 2.25 * ( float(ws_counter) / float(ws_sample_period))
	return ws
        
def serviceInterruptAnem():
        print "falling detected"
        ws_counter += 1
        print ws_counter
 
def gasSensor(v1,v2,c1,c2,c3):
        v1=(v1*1000)-c1
        v2=(v2*1000)-c2
        ppm=(v1-v2)/c3
        if ppm < 0:
                return 0
        return ppm

def sensorValNull():
        print(len(sensor_val))
        for num in range (0,17):
                sensor_val[num]=0.0
        #print(sensor_val)

def readShtTemp():
        try:
                return sht1x.read_temperature_C()
        except:
                return 0

def readShtHum():
        try:
                return sht1x.read_humidity()
        except:
                return 0

def sensorMeasure(number,interval):
        try:

                heaterControlOn()

                print("pump ON...")
                printlcd("Circulating...","Pump ON")
                GPIO.output(pump_pin,False)
                time.sleep(10.0)

                sensorValNull()
                for num in range (0,number):
                        printlcd("Measuring...","Sampling "+str(param[0]-num))
                        adc[0] = adc1.readADCSingleEnded(0, gain, sps) / 1000
                        adc[1] = adc1.readADCSingleEnded(1, gain, sps) / 1000
                        adc[2] = adc1.readADCSingleEnded(2, gain, sps) / 1000
                        adc[3] = adc1.readADCSingleEnded(3, gain, sps) / 1000
                        adc[4] = adc2.readADCSingleEnded(0, gain, sps) / 1000
                        adc[5] = adc2.readADCSingleEnded(1, gain, sps) / 1000
                        adc[6] = adc2.readADCSingleEnded(2, gain, sps) / 1000
                        adc[7] = adc2.readADCSingleEnded(3, gain, sps) / 1000
                        adc[8] = adc3.readADCSingleEnded(0, gain, sps) / 1000
                        adc[9] = adc3.readADCSingleEnded(1, gain, sps) / 1000
                        adc[10] = adc3.readADCSingleEnded(2, gain, sps) / 1000
                        adc[11] = adc3.readADCSingleEnded(3, gain, sps) / 1000
                        adc[12] = adc4.readADCSingleEnded(0, gain, sps) / 1000
                        adc[13] = adc4.readADCSingleEnded(1, gain, sps) / 1000
                        adc[14] = adc4.readADCSingleEnded(2, gain, sps) / 1000
                        adc[15] = adc4.readADCSingleEnded(3, gain, sps) / 1000
                        printADC()
                        sensor_val[0]=sensor_val[0]+gasSensor(adc[0],adc[1],c_sensor1[0],c_sensor1[1],c_sensor1[2])
                        sensor_val[1]=sensor_val[1]+gasSensor(adc[2],adc[3],c_sensor2[0],c_sensor2[1],c_sensor2[2])
                        sensor_val[2]=sensor_val[2]+gasSensor(adc[4],adc[5],c_sensor3[0],c_sensor3[1],c_sensor3[2])
                        sensor_val[3]=sensor_val[3]+gasSensor(adc[6],adc[7],c_sensor4[0],c_sensor4[1],c_sensor4[2])
                        sensor_val[4]=sensor_val[4]+gasSensor(adc[8],adc[9],c_sensor5[0],c_sensor5[1],c_sensor5[2])
                        sensor_val[5]=sensor_val[5]+measureCO2()
                        sensor_val[6]=sensor_val[6]+(adc[10]*1000/20)
                        # sensor_val[13]=sensor_val[13]+sht1x.read_temperature_C()
                        # sensor_val[14]=sensor_val[14]+sht1x.read_humidity()
                        sensor_val[13]=sensor_val[13] + readShtTemp()
                        sensor_val[14]=sensor_val[14] + readShtHum()
                        #sensor_val[16]=sensor_val[16]+getVaneDirection(adc[14])
                        sensor_val[16]=sensor_val[16]+adc[12] #cek pin ini benar atau tidak
                        print(param[0]-num)
                        time.sleep(interval)

                # averaging
                for num in range (0,17):
                        sensor_val[num]=sensor_val[num]/number
                
                heaterControlOff()

                print("pump OFF...")
                printlcd("Circulating...","Pump OFF")
                GPIO.output(pump_pin,True)
                
                print "sleep 1 min before PM measurement"
                time.sleep(60)

                measurePM()

                # sensor_val[15]= weatherStation.current_wind_speed()*1.60934
                sensor_val[15] = getWs()
                #sensor_val[16]=round(sensor_val[16],0)
                sensor_val[16]=getVaneDirection(sensor_val[16])
                
                print("------------------ result ----------------")    
                for num in range (0,17):
                        print(str(num)+"."+sensor_name[num]+"\t: "+str(sensor_val[num]))
        except:
                print ("Measurement Error")

def printADC():
        print"\tCH1(V)\t\tCH2(V)\t\tCH3(V)\t\tCH4(V)"
        print "ADC1=\t%0.3f\t\t%0.3f\t\t%0.3f\t\t%0.3f\t\t" % (adc[0],adc[1],adc[2],adc[3])
        print "ADC2=\t%0.3f\t\t%0.3f\t\t%0.3f\t\t%0.3f\t\t" % (adc[4],adc[5],adc[6],adc[7])
        print "ADC3=\t%0.3f\t\t%0.3f\t\t%0.3f\t\t%0.3f\t\t" % (adc[8],adc[9],adc[10],adc[11])
        print "ADC4=\t%0.3f\t\t%0.3f\t\t%0.3f\t\t%0.3f\t\t" % (adc[12],adc[13],adc[14],adc[15])

def readlineCR(port):
        rv = ""
        while True:
                ch = port.read()
                rv += ch
                if ch=='':
##            response=rv
                        return rv
##        else:
##            if ch!='\n' and ch!='\r':

def readlineCR1(port):
        rv = ""
        while True:
                ch = port.read()
                if ch=='\r' or ch=='':
                        return rv
                else:
                        if ch!='\n' and ch!='\r':
                                rv += ch

def readPM(port):
        rv = ""
        timeout_len = 0
        max_timeout = 20
        start = False
        bintang_counter = 0
        while True:
                ch = port.read()
                # gimana kalo pake counter ch ny ga ada karakter??? untuk buat expetion timeout
                if len(ch) != 0:
                        if ch == '*':
                                start = True
                                bintang_counter = bintang_counter + 1
                                if bintang_counter == 1: print "started"
                        if start == True:        
                                if bintang_counter == 2:
                                        return rv
                                else:
                                        rv += ch
                else:
                        timeout_len += 1
                        print "timeout: " + str(timeout_len)
                        if timeout_len == max_timeout:
                                return "error"

def uploadData():
        Nv=0
        for num in range(0,10):
                port1.write("N\r")
                Ndatum = readlineCR1(port1)
                #print(Ndatum)
                Nv = Nv + float(Ndatum)
##        print(num)
##        print(" ")
##        print(Nv)
                time.sleep(1)
 
        Nv = Nv/10   
##    port1.write("N\r")
##    Nv = readlineCR1(port1)
        CO2_val=Nv
        Nv = str(float(Nv))
        print("Konsentrasi:" + Nv)
        port1.write("V\r")
        Vv = readlineCR1(port1)
        Vv = Vv[1:]
        print("Lamp Voltage:" + Vv)
        port1.write("T\r")
        Tv = readlineCR1(port1)
        Tv = str(float(Tv))
        print("Temperature:" + Tv)
        time.sleep(5)
        return CO2_val
 
def measureCO2(): 
 try:     
        Nv=0
        if co2_status !=0: #preventing exception co2 sensor installation
                port1.write("N\r")
                Ndatum = readlineCR1(port1)
                if len(Ndatum)>0: #preventing null value
                        Nv = float(Ndatum)
                else:
                        Nv=-1
                        print("error: CO2 parsing failed")
        else:
                print("error: CO2 not installed")
                printlcd("Error","CO2 not installed")
                time.sleep(2)
                Nv = -2
        return Nv
 except:
  Nv = -2
 
def measurePM():
 try:
        if pm_status !=0: #preventing exception pm sensor installation
                port2.write("S\r")
                print("Command PM sent")
                printlcd("Measuring...","Command sent")
                # time.sleep(60)
                # response=port2.read(100)
                response = readPM(port2)
                print(response)
                print("Data Received " + str(len(response)))
                if (len(response)>50):
                        #response_s =response.strip('\n')
                        response_s=response.split(",")
                        #print type(response_s)
                        for num in range (0,6):
                                sensor_val[7+num]=float(response_s[2+num])
                        
                else:
                        port2.write("S\r")
                        print("Command PM resent")
                        printlcd("Measuring...","Command resent")
                        # time.sleep(80)
                        # response=port2.read(100)
                        response = readPM(port2)
                        #print(response)
                        print("Data Received " + str(len(response)))
                        if (len(response)>50):
                                #response_s =response.strip('\n')
                                response_s=response.split(",")
                                #print type(response_s)
                                for num in range (0,6):
                                        sensor_val[7+num]=float(response_s[2+num])
                        else:
                                print("Error data PM read")
                                printlcd("Measuring...","Error PM Read")
                                time.sleep(2)
 
        else:
                print("error: PM not installed")
                printlcd("Error","PM not installed")
                time.sleep(2)
                sensor_val[12]=-2.0 #marker err
 except:
  sensor_val[12]=-2.0
 
def getVaneDirection(value):
        print "vane pot value: " + str(value)
        if (value<=0.06) or (value> 3.1):
                return 0
        if (value>0.06) and (value<=0.23):
                return 1
        if (value>0.23) and (value<=0.43):
                return 2
        if (value>0.43) and (value<=0.63):
                return 3
        if (value>0.63) and (value<=0.8):
                return 4
        if (value>0.8) and (value<=0.99):
                return 5
        if (value>0.99) and (value<=1.1):
                return 6
        if (value>1.1) and (value<=1.4):
                return 7
        if (value>1.4) and (value<=1.68):
                return 8
        if (value>1.68) and (value<=1.9):
                return 9
        if (value>1.9) and (value<=2.1):
                return 10
        if (value>2.1) and (value<=2.3):
                return 11
        if (value>2.3) and (value<=2.5):
                return 12
        if (value>2.5) and (value<=2.7):
                return 13
        if (value>2.7) and (value<=2.9):
                return 14
        if (value>2.9) and (value<=3.1):
                return 15

def fuzzyCompare(compareValue, value):
        VARYVALUE = 0.05
        if ( (value > (compareValue * (1.0-VARYVALUE)))  and (value < (compareValue *(1.0+VARYVALUE))) ):
                return True
        return False

def heaterControlOn():
        if sensor_val[14]>heater_threshold:
                print ("Heater ON...")
                printlcd("Humid: "+str(sensor_val[14]),"Heater ON...")
                time.sleep(2)
                GPIO.output(heater_pin,True)

def heaterControlOff():
        print ("Heater OFF...")
        printlcd("Humid: "+str(sensor_val[14]),"Heater OFF...")
        time.sleep(2)
        GPIO.output(heater_pin,False)
 
def scroll(title,string,twolines):
    if twolines==1 and len(title)>0:
        print"error lenght"
        #lcd.close()
        exit()
    if len(title)>16:
        print"error exceed"
        #lcd.close()
        exit()
 
    longstr=len(string)
    for i in range (0,longstr):
        debut = i
        fin = debut+16
        line = string[debut:fin]
        if title and len(title)<17:
            lcd.lcd_display_string(title,1)
            #lcd.write_string(title)
            #lcd.cursor_pos=(1,0)
 
        lcd.lcd_display_string(line,2)
        #lcd.write_string(line)
        #lcd.cursor_pos=(1,0)
 
        if twolines==1:
            debut2=fin+1
            fin2=debut2+16
            line2 = string[debut2:fin2]
            #lcd.write_string(line2)
            lcd.lcd_display_string(line2,1)
 
        time.sleep(0.5)
        lcd.lcd_clear()
    #lcd.close()
       
def printlcd(text1,text2):
        lcd.lcd_clear()
        lcd.lcd_display_string(text1, 1)
        lcd.lcd_display_string(text2, 2)
        
def showlcd():
        print("Displaying...")
        lcd.lcd_clear()
        lcd.lcd_display_string("Displaying...", 1)
        lcd.lcd_display_string("Variable "+str(len(sensor_val)), 2)
        time.sleep(2)
 
        for num in range (0,8):
                #printlcd(str(sensor_name[num*2])+": "+str(sensor_val[num*2]),str(sensor_name[num*2+1])+": "+str(sensor_val[num*2+1]))
                #print(str(num)+"."+sensor_name[num]+"\t: "+str(sensor_val[num]))
                #time.sleep(3)
                printlcd((sensor_name[num*2]+(": %0.2f " % sensor_val[num*2])+sensor_unit[num*2]),(sensor_name[num*2+1]+(": %0.3f " % sensor_val[num*2+1])+sensor_unit[num*2+1]),)
                time.sleep(3)

        printlcd(str(sensor_name[16])+": "+str(sensor_val[16]),"Counter: "+str(i))
        time.sleep(3)
 
signal.signal(signal.SIGINT, signal_handler)

#print 'Press Ctrl+C to exit'
print(param[0])
print(param[1])
time.sleep(4.0)
d=datetime.now()
date= "{:%Y-%m-%d}".format(d)
time_current= "{:%H:%M:%S}".format(d)
 
printlcd("Sensync Base V2","Welcome Back...")
time.sleep(3)
printlcd(date,time_current)
print(date)
print(time_current)

print("-----------------------------------")
print("Sensync Base V2")
print(" ")

# GPIO.output(valve_b_pin,False)
# time.sleep(3.0)
# printlcd("Checking...","Pump ON...")
# print("Pump ON...")
# GPIO.output(pump_pin,False)
# time.sleep(2.0)
# print("Pump OFF...")
# printlcd("Checking...","Pump OFF...")
# GPIO.output(pump_pin,True)
# time.sleep(2.0)
 
#start report
try:
	report_start=urllib.urlopen(url_start+id)
	response_report_start=report_start.read()
	print(response_report_start)
	printlcd("Start Report",response_report_start)
except:
	print('Network Error')
	printlcd("Start Report","Network Error")

# Read channel 0 in single-ended mode using the settings above
while True:
        
        sensorMeasure(param[0],param[1])

        d=datetime.now()
        date= "{:%Y-%m-%d}".format(d)
        time_c= "{:%H:%M:%S}".format(d)
        # filename="/home/airpowerx11/data/"+"{:%Y%m%d}".format(d)+".txt"
        filenamefd="/media/fd/"+"{:%Y%m%d}".format(d)+".txt"
        
        print(date)
        print(time_c)
        printlcd(date,time_c)
        time.sleep(2.0)
        #filename="log_adc_8sps_co2_pm_gas.txt"
         
	#ppm to ug/m3 conversion here
        sensor_val[0]=sensor_val[0]*1000*2.00
        sensor_val[1]=sensor_val[1]*1000*1.145
        sensor_val[2]=sensor_val[2]*1000*1.88
        sensor_val[3]=sensor_val[3]*1000*2.62
        sensor_val[4]=sensor_val[4]*1000*1.25		 
	
        #upload data
        # gm is GET Method
        val0_gm='?o3='+str(sensor_val[0])
        val1_gm='&co='+str(sensor_val[1])
        val2_gm='&no2='+str(sensor_val[2])
        val3_gm='&so2='+str(sensor_val[3])
        val4_gm='&no='+str(sensor_val[4])
        val5_gm='&co2='+str(sensor_val[5])
        val6_gm='&voc='+str(sensor_val[6])
        val7_gm='&pm1='+str(sensor_val[7])
        val8_gm='&pm25='+str(sensor_val[8])
        val9_gm='&pm4='+str(sensor_val[9]) 
        val10_gm='&pm10='+str(sensor_val[10])
        val11_gm='&tsp='+str(sensor_val[11])
        val12_gm='&err='+str(sensor_val[12])
        temp = int(round(sensor_val[13], 0))
        val13_gm='&temp='+str(sensor_val[13])
        hum = int(round(sensor_val[14], 0))
        val14_gm='&hum='+str(sensor_val[14])
        val15_gm='&ws='+str(sensor_val[15])
        val16_gm='&wd='+str(sensor_val[16])
        id_gm='&id='+id
        #printlcd("upload data...")
        #time.sleep(2.0)
        print (url_upload+val0_gm+val1_gm+val2_gm+val3_gm+val4_gm+val5_gm+val6_gm+val7_gm+val8_gm+val9_gm+val10_gm+val11_gm+val12_gm+val13_gm+val14_gm+val15_gm+val16_gm+id_gm)
        try:
                # http://secure.getsensync.com/proc/bplhd_jabar/base/process.php?03=0.1&co2=0.2&no2=0.3&so2=0.4&no=0.5&co2=0.6&voc=0.7&pm1=0.8&pm25=0.9&pm4=1.0&pm10=1.1&tsp=1.2&err=0&temp=27&hum=75&ws=13&wd=2.5&id=x-bplhd-jabar-dpk
        	print "trying to upload"
                data_upload = url_upload+val0_gm+val1_gm+val2_gm+val3_gm+val4_gm+val5_gm+val6_gm+val7_gm+val8_gm+val9_gm+val10_gm+val11_gm+val12_gm+val13_gm+val14_gm+val15_gm+val16_gm+id_gm
                upload=urllib.urlopen(data_upload)
        	response_upload=upload.read()
        	print(response_upload)
		printlcd("Data Upload",response_upload)
        except socket.timeout:
                print "timeout"
                printlcd("Data Upload","timeout")
        except:
                print("Network Error")
                writelog("network error")	
                printlcd("Data Upload","Network Error")

        time.sleep(2.0)
         
        print("saving data..."+filenamefd)
        # f=open(filename, 'a')
        # #f.write(volts5+","+volts6+","+volts7+","+volts8+"\n")
        # #f.write("%0.0f,%0.3f,%0.3f,%0.3f,%0.3f,%0.2f,%0.2f,%0.2f,%0.2f,%0.0f," % (i,volts5,volts6,volts7,volts8,CO2_val,temp,hum,currentWindSpeed,wd_val)+response_s[2]+","+response_s[3]+","+response_s[4]+","+response_s[5]+","+response_s[6]+","+response_s[7]+",\n")
        # f.write(date+" "+time_c+",%0.0f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.0f," % (i,sensor_val[0],sensor_val[1],sensor_val[2],sensor_val[3],sensor_val[4],
        #          sensor_val[5],sensor_val[6],sensor_val[7],sensor_val[8],sensor_val[9],
        #          sensor_val[10],sensor_val[11],sensor_val[12],sensor_val[13],sensor_val[14],
        #          sensor_val[15],sensor_val[16])+",\n")
        # f.close()
        f=open(filenamefd, 'a')
        #f.write(volts5+","+volts6+","+volts7+","+volts8+"\n")
        #f.write("%0.0f,%0.3f,%0.3f,%0.3f,%0.3f,%0.2f,%0.2f,%0.2f,%0.2f,%0.0f," % (i,volts5,volts6,volts7,volts8,CO2_val,temp,hum,currentWindSpeed,wd_val)+response_s[2]+","+response_s[3]+","+response_s[4]+","+response_s[5]+","+response_s[6]+","+response_s[7]+",\n")
        f.write(date+" "+time_c+",%0.0f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.0f," % (i,sensor_val[0],sensor_val[1],sensor_val[2],sensor_val[3],sensor_val[4],
                 sensor_val[5],sensor_val[6],sensor_val[7],sensor_val[8],sensor_val[9],
                 sensor_val[10],sensor_val[11],sensor_val[12],sensor_val[13],sensor_val[14],
                 sensor_val[15],sensor_val[16])+",\n")
        f.close()		
        print("data saved")
        printlcd("Saving Data...",filenamefd)
        time.sleep(2.0)
        c=open('lcd.txt','w')
        c.write(date+" "+time_c+",%0.0f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.0f," % (i,sensor_val[0],sensor_val[1],sensor_val[2],sensor_val[3],sensor_val[4],
                 sensor_val[5],sensor_val[6],sensor_val[7],sensor_val[8],sensor_val[9],
                 sensor_val[10],sensor_val[11],sensor_val[12],sensor_val[13],sensor_val[14],
                 sensor_val[15],sensor_val[16])+",\n")
        c.close()
        
        i+=1

        showlcd()
        print "sleep"
        time.sleep(60*25)
