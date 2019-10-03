from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
from pymavlink import mavutil
import RPi.GPIO as GPIO                    #Import GPIO library
import socket
import exceptions  
import math
import argparse


GPIO.setmode(GPIO.BCM)                     #Set GPIO pin numbering 

TRIG = 23                                  #Associate pin 23 to TRIG
ECHO = 24                                  #Associate pin 24 to ECHO

print ("Distance measurement in progress")

GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out
GPIO.setup(ECHO,GPIO.IN)                   #Set pin as GPIO in



def connectMyCopter():
    
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    vehicle = connect(connection_string,wait_ready=True)
    return vehicle

vehicle = connectMyCopter()
vehicle.mode = VehicleMode("LAND") 
time.sleep(15)                      #Sleep time required for copter to land
vehicle.close()      
