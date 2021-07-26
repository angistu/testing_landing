"""
NOTE: be sure to be using the latest dronekit. 
sudo pip uninstall dronekit
sudo pip uninstall pymavlink
cd dronekit-python
git pull
sudo python setup.py build
sudo python setup.py install
Be sure the RASPI CAMERA driver is correctly acivated -> type the following
modprobe bcm2835-v4l2 
"""
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import time
import math
import argparse


from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from opencv.lib_aruco_pose import *

parser = argparse.ArgumentParser(description='Precise Land')
parser.add_argument('--connect', default = '/dev/ttyAMA0')
args = parser.parse_args()

connection_string = args.connect
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

print ("Autopilot Firmware version: %s" % vehicle.version)
print ("Global Location: %s" % vehicle.location.global_frame)
print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print ("Local Location: %s" % vehicle.location.local_frame) #NED
print ("Attitude: %s" % vehicle.attitude)
print ("Velocity: %s" % vehicle.velocity)
print ("GPS: %s" % vehicle.gps_0)
print ("Groundspeed: %s" % vehicle.groundspeed)
print ("Airspeed: %s" % vehicle.airspeed)
print ("Battery: %s" % vehicle.battery)
print ("EKF OK?: %s" % vehicle.ekf_ok)
print ("Last Heartbeat: %s" % vehicle.last_heartbeat)
print ("Heading: %s" % vehicle.heading)
print ("Is Armable?: %s" % vehicle.is_armable)
print ("System status: %s" % vehicle.system_status.state)
print ("Mode: %s" % vehicle.mode.name) # settable
print ("Armed: %s" % vehicle.armed) # settable

#--------------------------------------------------
#-------------- ARM AND TAKEOFF  
#--------------------------------------------------   

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks ... DON'T TOUCH!!!")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for pidrone to initialise...")
        time.sleep(1)

        
    print("Arming Motors")
    # VTOL should arm in QLOITER mode
    print("Switch mode To GUIDED")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print("Waiting For Arming...")
        time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached Target Altitude")
            break
        time.sleep(1)

#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------    

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
    print "dlat, dlon", dLat, dLon

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.



def adds_locations(aLocation): #target locations
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	
    
    cmds = vehicle.commands

    print("Clear any existing commands")
    cmds.clear() 
    
    print("Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = LocationGlobalRelative(-6.9802862, 107.5700757)
    point2 = LocationGlobalRelative(-6.9801824, 107.5697377)
    point3 = LocationGlobalRelative(-6.9801291, 107.5694320)
    point4 = LocationGlobalRelative(-6.9801291, 107.5694320)
   
    #cmds.add(command(0, 0, 0, target component,seq,frame,command,current,autocontionue,param1,param2,param3,param4                 ,     x     ,     y     ,z ))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 10)) #angka terakhir altitude target
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 10))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 10))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point4.lon, 10))
   
    print("Upload new commands to vehicle")
    cmds.upload()

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
    
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)
        
#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
#print('Connecting...')
#vehicle = connect(args.connect)  

#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
id_to_find      = 1
marker_size     = 100 #- [cm]
freq_send       = 1 #- Hz

land_alt_cm         = 50.0
angle_descend       = 20*deg_2_rad
land_speed_cms      = 30.0

#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
#cwd                 = path.dirname(path.abspath(__file__))
calib_path          = "home/pi/PycharmProjects/aruco/camera_01/"
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')                                      
aruco_tracker       = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False, 
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)

#------------- MAIN FUNCTION
print('Create a new mission (for current location)')
adds_locations(vehicle.location.global_frame)

#------------- MISSION 1
arm_and_takeoff(10)
print ("Take Off Complete")

#-- set the default speed
print ("Set Target airspeed to 7")
vehicle.airspeed = 7

for i in range(3, -1, -1): #counting down hover time before do mission
    time.sleep(1) #3sec
    print('Count To Go: '+ str(i))

print('Fly')

print("Starting Mission 1")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0 #target waypoint

# Set mode to AUTO to start mission
print("Switch mode to AUTO")
vehicle.mode = VehicleMode("AUTO")

# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.

while True:
    nextwaypoint=vehicle.commands.next
    print('Distance to waypoint (%s): %s Altitude: %s' % (nextwaypoint, distance_to_current_waypoint(), vehicle.location.global_relative_frame.alt))
    time.sleep(1)
    if vehicle.commands.next==3: #dummy waypoint for landing near waypoint 2
        print('Land and Drop')
        break


#---------------- PRECISION LANDING
time_0 = time.time()

while True:                

    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    if marker_found:
        x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
        uav_location        = vehicle.location.global_relative_frame
        
        #-- If high altitude, use baro rather than visual
        if uav_location.alt >= 5.0:
            print 
            z_cm = uav_location.alt*100.0
            
        angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)

        
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            # print ""
            print " "
            print "Altitude = %.0fcm"%z_cm
            print "Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg)
            
            north, east             = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
            print "Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, vehicle.attitude.yaw*rad_2_deg)
            
            marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01)  
            #-- If angle is good, descend
            if check_angle_descend(angle_x, angle_y, angle_descend):
                print "Low error: descending"
                location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.01/freq_send))
            else:
                location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                
            vehicle.simple_goto(location_marker)
            print "UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon)
            print "Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon)
            
        #--- COmmand to land
        if z_cm <= land_alt_cm:
            if vehicle.mode == "GUIDED":
                print (" -->>COMMANDING TO LAND<<")
                vehicle.mode = "LAND"
                break

#-------------------- SERVO RELEASE
#To control a servo, plug it into an empty channel on the pixhawk and use
#mission planner to set that channel as a servo channel.
#From dronekit, control it like this:
msg = vehicle.message_factory.command_long_encode(
0, 0,    # target_system, target_component
mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
0, #confirmation
8,    # servo number
1700,          # servo position between 1000 and 2000
0, 0, 0, 0, 0)    # param 3 ~ 7 not used

# send command to vehicle
print("Release Package Drop")
vehicle.send_mavlink(msg)

#----------------------------------misi 2-----------------------------------#

for i in range(5, -1, -1): #counting down time in ground before take off
    time.sleep(1) #5sec
    print('Count To Take Off: '+ str(i))

arm_and_takeoff(10)
print ("Take Off Complete")

#-- set the default speed
print ("Set Target airspeed to 7")
vehicle.airspeed = 7

for i in range(3, -1, -1): #counting down hover time before do mission
    time.sleep(1) #3sec
    print('Count To Go: '+ str(i))

print('Fly')

print('Return to launch')
print("Switch mode to RTL")
vehicle.mode = VehicleMode("SMART_RTL") #default altitude rtl 15m
while True:
    vehicle.location.global_relative_frame.alt is not 0.5
    print("Return to home, Altitude: %s" % vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt <= 0.5 : #point to vehicle.close
        print("Landed")
        print("DISARMING MOTORS")
        break
    time.sleep(1)

#Close vehicle object before exiting script
print("Congratulation! Mission Complete")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
