from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)



def condition_yaw(heading, relative=False):
    
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)





def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)




#Arm and take of to altitude of 3 meters
arm_and_takeoff(3)

try:
	
	DURATION = 20 #Set duration for each segment.
	#Set up velocity vector to map to each direction.
	# vx > 0 => fly North
	# vx < 0 => fly South
	NORTH = 2
	SOUTH = -2

	# Note for vy:
	# vy > 0 => fly East
	# vy < 0 => fly West
	EAST = 2
	WEST = -2

	# Note for vz: 
	# vz < 0 => ascend
	# vz > 0 => descend
	UP = -1
	DOWN = 1


	# Square path using velocity
	print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and velocity parameters")

	print("Yaw 180 absolute (South)")
	condition_yaw(180)

	print("Velocity South & up")
	send_ned_velocity(SOUTH,0,UP,DURATION)
	send_ned_velocity(0,0,0,1)


	print("Yaw 270 absolute (West)")
	condition_yaw(270)

	print("Velocity West & down")
	send_ned_velocity(0,WEST,DOWN,DURATION)
	send_ned_velocity(0,0,0,1)


	print("Yaw 0 absolute (North)")
	condition_yaw(0)

	print("Velocity North")
	send_ned_velocity(NORTH,0,0,DURATION)
	send_ned_velocity(0,0,0,1)


	print("Yaw 90 absolute (East)")
	condition_yaw(90)

	print("Velocity East")
	send_ned_velocity(0,EAST,0,DURATION)
	send_ned_velocity(0,0,0,1)



except KeyBoardInterrupt

       print("Setting LAND mode...")
       vehicle.mode = VehicleMode("LAND")


       #Close vehicle object before exiting script
       print "Close vehicle object"
       vehicle.close()
       print("Completed")


