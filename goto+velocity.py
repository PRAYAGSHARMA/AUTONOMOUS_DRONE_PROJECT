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
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
   

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_local_ned_velocity(vx,vy,vz,duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions
        vx,vy,vz, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(10)    

#The code ~ python connection_template.py --connect <ip>:<port>
vehicle = connectMyCopter()

arm_and_takeoff(2)

try:
  
  print("Set default/target airspeed to 3")
  vehicle.airspeed = 2
  
  print("Going towards first point for 30 seconds ...")
  point1 = LocationGlobalRelative(30.7483577756501,76.757645085454, 3)
  vehicle.simple_goto(point1)
  
  # sleep so we can see the change in map
  time.sleep(15)
  

  
  
  print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
  point2 = LocationGlobalRelative(30.7483698776707,76.7575585842133, 3)
  vehicle.simple_goto(point2, groundspeed=3)
  
  # sleep so we can see the change in map
  time.sleep(15)
  
  print("moving in + x")    
  send_local_ned_velocity(1,0,0,2)

  
  vehicle.mode    = VehicleMode("LAND")
  print("LAND")
  time.sleep(15)
  vehicle.close()   
      
      
      


except KeyboardInterrupt:
  GPIO.cleanup()
  vehicle.mode    = VehicleMode("LAND")
  print("LAND")
  time.sleep(20)
  vehicle.close()
  


































