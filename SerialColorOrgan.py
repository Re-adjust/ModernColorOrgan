import matplotlib.pyplot as plt
import numpy as np

import time
import sys
import signal
import serial

def Exit_gracefully(signal,frame):
  ser.flush()
  ser.close()             # close port
  print('Exiting on sigint')
  plt.close()
  sys.exit(0)

def ReadLine():
  line=ser.readline().decode('utf-8')


  while len(line)==0:
    line=ser.readline().decode('utf-8')

  return line

signal.signal(signal.SIGINT, Exit_gracefully)
ser = serial.Serial('COM8',9600,timeout=10)  # open serial port
print(ser.name)         # check which port was really used
ser.flush()

line=ReadLine()
print(line)

##line=ser.readline()
##while len(line)==0:
##  line=ser.readline()

while True:
  FFTVal=[];
  Ct=[];
  linect=0;
  ser.write(b'DATA\n')     # write a string
     
  for i in range(64):
    line=ReadLine()
    print (i,"\t",line,)
    Ct.append(i);
    FFTVal.append(float(line));
    
  plt.plot(Ct,FFTVal) 
  plt.show()     


