#!/usr/bin/python

#SensyncBase v.1.1.11
#Get Sensor Data from Particulate sensor, CO2 sensor,
#Temperature and Humidity Sensor
#bubaka@20170717

import serial
import datetime
import time
import os
import subprocess
import re
import sys

PMstatus  = 1
CO2status = 1
ACMstatus = 1

def logo():
	print "-----------------------------------------------------------------------------------------"
	print "     .:::::::::.      "                                                                  
	print "   '::::::--:::::'    "                                                                  
	print "  ':::-'''--''-:::'   "                                                                  
	print "  ':::  .::::. `:::'    .:::.  .:::.   ::.::.   .:::. ':'  ':' ::.:::.   .:::.   "                                                                 
	print "  ':::  :..:::  :::'   ::  '' ::   ::  ::''':: ::  ''  ::..::  ::'''::  ::'  ''  "                                                                
	print "  ':::.  '::' .:::'    '::::. :::::::  ::   :: '::::.   ::::   ::   ::  ::       "                                                               
	print "   '::::......::::'    ..  :: :::      ::   :: ..  ::    ::    ::   ::  ::.  ..  "                                                                
	print "    .::::::::::::.     '::::'  :::::'  ::   :: '::::'    ::    ::   ::   ':::'   "       
	print "     .:::....:::.     "       
	print "     ':::::::::'       %%%%%   %%%%   %%%%% %%%%%%"
	print "      '::...::'        %%  %% %%  %% %%     %%        "
	print "        '::::'         %%%%%  %%%%%%  %%%%  %%%%%     0 0  0 0   0   0    0  000   0  "
	print "         '::'          %%  %% %%  %% 	 %% %%        0    000  000  0    0  00   000 "                        
	print "          ''           %%%%%  %%  %% %%%%%  %%%%%%    0 0  0 0  0 0  000  0  0    0 0 "                        
	print "        .::::.                  "                                                        
	print "         ''''           (c)2017 "
	print "-----------------------------------------------------------------------------------------"


#Syntax: progressbar(range, "Progress: ", width):
def progressbar(it, prefix="", size=60):
    count = len(it)
    def _show(_i):
        x = int(size*_i/count)
        sys.stdout.write("%s[%s%s] %i/%i\r" % (prefix, "#"*x, "."*(size-x), _i, count))
        sys.stdout.flush()

    _show(0)
    for i, item in enumerate(it):
        yield item
        _show(i+1)
    sys.stdout.write("\n")
    sys.stdout.flush()
		
#-- function to map the port of CO2 sensor
def mappingCO2():
	ttyCO2 = "0"
	dmesg = subprocess.check_output('dmesg | grep tty', shell = True)
        dmesg = dmesg.split("\n")
	n = len(dmesg)-1
	while n >= 0:
		if dmesg[n].find("FTDI USB Serial Device converter now attached to") >= 0:
			ttyCO2=dmesg[n].split(" ")
			ttyCO2="/dev/"+ttyCO2[len(ttyCO2)-1]		
			return ttyCO2
			break
		elif dmesg[n].find("FTDI USB Serial Device converter now disconnected from") >= 0:
			ttyCO2 = "0"
			print ("CO2 sensor USB connection disconnected")
			break
		n = n-1
	return ttyCO2

#-- function to map the port of PM sensor
def mappingPM():
	ttyPM = "0"
	dmesg = subprocess.check_output('dmesg | grep tty', shell = True)
        dmesg = dmesg.split("\n")
	n = len(dmesg)-1
	while n >= 0:
		if dmesg[n].find("cp210x converter now attached to") >= 0:
			ttyPM=dmesg[n].split(" ")
			ttyPM="/dev/"+ttyPM[len(ttyPM)-1]		
			return ttyPM
			break
		elif dmesg[n].find("cp210x converter now disconnected from") >= 0:
			ttyPM = "0"
			print ("PM sensor USB connection disconnected")
			break
		n = n-1
	return ttyPM

#-- function to map the port of Arduino multi gas and T/RH sensors 
def mappingACM():
	ttyACM = "0"
	dmesg = subprocess.check_output('dmesg | grep tty', shell = True)
        dmesg = dmesg.split("\n")
	n = len(dmesg)-1
	while n >= 0:
		if dmesg[n].find("ttyACM") >= 0:
			ttyACM=dmesg[n]
			ttyACM=ttyACM[ttyACM.find("ttyACM"):ttyACM.find("ttyACM")+7]
			ttyACM="/dev/"+ttyACM		
			return ttyACM
			break
		n = n-1
	return ttyACM
	
#-- readline function to read from serial
def rdln(port):
        ro = ""
        while True:
                rn = port.read()
                ro += rn
                if rn=='':
                        return ro

#-- check wheter output of the sensor is float
def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

#-- dictionary to convert month's name to number
def month(x):
	return {
		'JAN': '01',
		'FEB': '02',
		'MAR': '03',
		'APR': '04',
		'MAY': '05',
		'JUN': '06',
		'JUL': '07',
		'AUG': '08',
		'SEP': '09',
		'OCT': '10',
		'NOV': '11',
		'DEC': '12',
	} [x]

#-- serial port mapping
ttyPM  = mappingPM()
ttyCO2 = mappingCO2()
ttyACM = mappingACM()
ttyATH = "/dev/ttyATH0"

#-- Date Time Config
tstmp = time.time()
timestamp = datetime.datetime.fromtimestamp(tstmp).strftime('%Y-%m-%dT%H:%M:%S')
dtime = datetime.datetime.fromtimestamp(tstmp).strftime('%Y-%m-%d %H:%M:%S')

#-- Time Date parsing
settime = dtime.replace("-"," ")
settime = settime.replace(":"," ")
settime = settime.replace("20","")
settime = settime[:-2]	#remove detik
#print (settime)
#print

#===========================================================
subprocess.call('clear') #clear screen
print ("=== Sensync Base Chalifa v1.1.1 ===")
logo()
print
print (dtime)

#-- check and define serial port for Particulate sensor
try:	
	print('Connecting to sensor module...')
	PMport = serial.Serial(ttyPM,38400, timeout=3.0)
	PMportcheck = PMport.isOpen()
	PMstatus = 1
	print('PMmeter...    Connected'), ttyPM
except (serial.serialutil.SerialException,OSError), error:
	PMstatus = 0
	print('PMmeter Disconnected')
	print (error)

#-- check and define serial port for CO2 sensor
try:	
	CO2port = serial.Serial(ttyCO2,19200, timeout=3.0)
	CO2portcheck = CO2port.isOpen()
	CO2status = 1
	print("CO2 Sensor... Connected"), ttyCO2
except (serial.serialutil.SerialException,OSError), error:
	CO2status = 0
	print("CO2 Sensor Disconnected!")
	print (error)

#-- check and define serial port for Arduino
try:	
	ACMport = serial.Serial(ttyACM,9600,timeout=3.0)
	ACMportcheck = ACMport.isOpen()
	ACMstatus = 1
	print("Arduino...    Connected"), ttyACM
except (serial.serialudmetil.SerialException,OSError), error:
	ACMstatus = 0
	print("Arduino Disconnected")
	print (error)

#-- check and define serial port for Serial GLiNet
try:	
	ATHport = serial.Serial(ttyATH,115200,timeout=3.0)
	ATHportcheck = ATHport.isOpen()
	ATHstatus = 1
	print("Serial Debug...   Ready"), ttyATH
except: 
	ATHstatus = 0
	print("Serial Debug...  None")
	
#-- get data from Arduino
if ACMstatus != 0:
	print
	ACMport.write("HELP\r")
	time.sleep(2)
	response=rdln(ACMport)
	print(response)
	if response.find("OK") >= 0:
		print			
		print("Request Sensor Data")
		ACMport.write("SENSOR\r")
		#print("Please wait...")
		#time.sleep(5)			
		print("Load T/RH and WIND data...")
		for i in progressbar(range(10), "", 10):
			time.sleep(1)
		print
		print("Load Gas Sensor data...")
		for i in progressbar(range(35), "", 35):
			time.sleep(1)
		print
		print("Wait for display...")
		for i in progressbar(range(15), "", 15):
			time.sleep(1)
		response=ACMport.readlines()
		#response=rdln(ACMport)
		#print(response)
		#print('Start sorting data...') #debug only
		if response[0].find("Get data from all Sensor") >= 0:			
			data = response						#reading the output
			for i in xrange(0,len(data)-1):		#start sorting data
				if data[i].find("SHT:") >=0:
					SHT=data[i]
				if data[i].find("WIND:Speed") >=0:
					WIND=data[i+1]
				if data[i].find("GAS VALUE (ppm)") >=0:
					GASPPM=data[i+1]
				if data[i].find("GAS VALUE =>") >=0:
					GASVAL=data[i+1]
			#-- print data log
			print
			print ("SHT11"), SHT
			print ("WIND"), WIND
			print ("GASPPM")
			print (GASPPM)
			print ("GASVAL")
			print (GASVAL)
			#-- filter data		
			SHT=re.findall(r"[-+]?\d*\.\d+|\d+", SHT)			
			#TMP=re.findall(r"[-+]?\d*\.\d+|\d+", TMP)			
			WIND=re.findall(r"[-+]?\d*\.\d+|\d+", WIND)		
			GASPPM=re.findall(r"[-+]?\d*\.\d+|\d+", GASPPM)
			GASVAL=re.findall(r"[-+]?\d*\.\d+|\d+", GASVAL)	
			
			f = file('dataSHTWIND.csv','a',os.O_NONBLOCK)
			f.write(timestamp+","+SHT[0]+","+SHT[1]+","+WIND[0]+","+WIND[1]+",W\n")
			f.close
			f = file('dataGASPPM.csv','a',os.O_NONBLOCK)
			f.write(timestamp+","+GASPPM[1]+","+GASPPM[2]+","+GASPPM[3]+","+GASPPM[5]+","+GASPPM[7]+","+GASPPM[9]+",W\n")
			f.close
			f = file('dataGASVAL.csv','a',os.O_NONBLOCK)
			f.write(timestamp+","+GASVAL[0]+","+GASVAL[2]+","+GASVAL[3]+","+GASVAL[5]+","+GASVAL[7]+","+GASVAL[9]+",W\n")
			f.close
			print('Sensor Data Record... Done')			
		else:
			print('Sensor output does not follow the format. . .')
		
	else:
		print('ACM multi gas sensor problem initiating command . . .')
else:
	print('ACM multi gas sensor problem connection...')	

#-- get data from PM meter and CO2 sensor
if PMstatus != 0:
	print 
	print("Send request to PMmeter...")
	ACMport.write("PM10\r") 	#show status on LCD
	time.sleep(1)
	PMport.write("1\r")
	time.sleep(1)
	response=rdln(PMport)	
	print("Load PMmeter data...")
	if response.find("Settings Report") >= 0:	
		PMport.write("S\r")
		#time.sleep(1)
		response=rdln(PMport)
		if response.find("Start") >= 0:
			#time.sleep(75)		
			for i in progressbar(range(65), "", 65):
				time.sleep(1)
			dataPM = PMport.readline()	#reading the output
			dataPM = dataPM.lstrip().rstrip()	#clean delimiter
			dataPM = dataPM.split(",")
			#-- extract timestamp
			timestamp = dataPM[0].split(" ")
			date = timestamp[0].split("/")
			hms = timestamp[1] #hms = hour-minute-second	
			dd = date[0]
			mm = month(date[1])
			yyyy = date[2]
			timestamp = yyyy+"-"+mm+"-"+dd+"T"+hms 
			#timestamp = datetime.datetime.now().isoformat()
						
			PMport.write("4 1\r")
			time.sleep(1)
			response=rdln(PMport)
			print(response)
			print("PMmeter Data Record... Done")
			print
		else:
			print('Sensor does not start . . .')
			
		#-- get data from CO2 sensor
		print("Read CO2 Sensor")
		dataCO2 = 0
		if CO2status != 0:
			while True:			
				CO2port.write("N\r")	
				dataCO2 = CO2port.readline()
				dataCO2 = dataCO2.rstrip().lstrip()	
				if is_number(dataCO2) == True:
					break
		else:
			print('CO2 sensor problem connection . . .')
		print ("CO2:"),dataCO2, ("ppm") 
		print("NDIR CO2 Data Record... Done")
		
		#-- Save to log
		f = file('dataPMCO2.csv','a',os.O_NONBLOCK)
		f.write(timestamp+","+dataPM[2]+","+dataPM[3]+","+dataPM[4]+","+dataPM[5]+","+dataPM[6]+","+dataCO2+",W\n")
		f.close
		
		dataPM = dataPM[2]
		#print "Sensync",u"\u00a9","2016"				
	else:
		print('PM sensor problem initiating command . . .')
else:
	print('PM sensor problem connection . . .')
	#-- Closing port
	PMport.close()

#======================================== Upload data to server
print("Send data PM + CO2 to Arduino...")
ACMport.write("PMCO2 " + dataPM + " " + dataCO2 + "\r")
time.sleep(2)
response=ACMport.readlines()
print (response[0])
print (response[1])
ACMport.write("UPLOAD\r")
print
print("Data Uploading...")
print("Initalize...")
for i in progressbar(range(40), "", 40):
	time.sleep(1)
print
print("Upload to Sensync Server...")
for i in progressbar(range(60), "", 60):
	time.sleep(1)
print
response=rdln(ACMport)
#response=ACMport.readlines()
if response.find("failed") >= 0:
	print("Upload Failed!")
if response.find("SEND OK") >= 0:
	print("Upload OK")
print

print("Upload to Sensync Server R3...")
for i in progressbar(range(60), "", 60):
	time.sleep(1)
print
response=rdln(ACMport)
#response=ACMport.readlines()
if response.find("failed") >= 0:
	print("Upload Failed!")
if response.find("SEND OK") >= 0:
	print("Upload OK")
print

print("Upload to Nuestodev Server...")
for i in progressbar(range(60), "", 60):
	time.sleep(1)
print

response=rdln(ACMport)
#response=ACMport.readlines()
if response.find("failed") >= 0:
	print("Upload Failed!")
if response.find("SEND OK") >= 0:
	print("Upload OK")
	
#-- Closing port
ACMport.close()

#-- End of Code
print ("-----")
print
