#!/usr/bin/env python
#####################################
#           Simurg Takimi           #
#            Produced by:           #
#         Yusuf Islam Hamamci       #
#          Mehmet Recep Askar       #
#           Melih Gazi Ulker        #
#                                   #
#####################################
from __future__ import print_function
import sys
from pid_aci import PID
import cv2
import numpy as np
import time
from ip import baslat,islet
from goruntu_isle1 import baslat,hangisi
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
import argparse

def gstreamer_pipeline() :   
	return ('nvarguscamerasrc ! ' 
	'video/x-raw(memory:NVMM), '
	'width=(int)1280, height=(int)720, '
	'format=(string)NV12, framerate=(fraction)24/1 ! '
	'nvvidconv flip-method=2 ! '
	'video/x-raw, width=640, height=(int)480, format=(string)BGRx ! '
	'videoconvert ! '
	'video/x-raw, format=(string)BGR ! appsink')

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

pist_1=[1,1]
pist_2=[-1,1]
pist_3=[-1,-1]
pist_4=[1,-1]
pist_5=[0,0]

pid = PID(0.001,0,0)
train_descriptor=[]
score = [0,0,0,0,0,0,0,0,0,0]
alpha = 0
stm_pist = None
odtu_pist=None
ort_pist=None
landing_pist=None


for each in range(5):
    thresh, dedet, isim, sift, train_descriptor_a, surf, bf = baslat(each)
    train_descriptor.append(train_descriptor_a)

parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = '/dev/ttyACM0'
sitl = None

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True



    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
    the target position. This allows it to be called with different position-setting commands.
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """

    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.3: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;
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
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)

def position( pos_x, pos_y, pos_z, heading):

     msg = vehicle.message_factory.set_position_target_local_ned_encode(
              0,
              0, 0,
              mavutil.mavlink.MAV_FRAME_LOCAL_NED,
              0b100111111000,
              pos_x, pos_y, -pos_z,
              0, 0, 0,
              0, 0, 0,
              heading, 0)

     vehicle.send_mavlink(msg)

arm_and_takeoff(1)
print("Set groundspeed to 3m/s.")
vehicle.groundspeed=1
print("Position North 4 East 4")

position(1, 1,1,0)

while True:
    print(vehicle.location.local_frame.east,vehicle.location.local_frame.north)
    if(0.5<vehicle.location.local_frame.east<1.5 and 0.5<vehicle.location.local_frame.north<1.5):
        break

while True :
    baslangic = time.time()
    ret,cv_image = cap.read()
    cv_image = cv2.resize(cv_image,(640,480), interpolarization = cv2.INTER_AREA)
    karar, alpha, score =  hangisi(cv_image,thresh,dedet,isim,sift,train_descriptor,surf,bf,alpha,score)
    if karar != None :
        whichone = karar
        alpha = 0
        score = [0,0,0,0,0,0,0,0,0,0]
    if whichone=='stm':
        stm_pist = pist_1
        whichone=None
        print('1.pistde stm')
        break
    elif whichone=='odtu':
        odtu_pist = pist_1
        whichone=None
        print('1.pistde odtu')
        break
    elif whichone=='ort':
        ort_pist=pist_1
        whichone= None
        print('1.pistde ort')
        break
    elif whichone=='landing':
        landing_pist=pist_1
        whichone= None
        print('1.pistde landing')
        break
    bitis = time.time()
    fps=1/(bitis-baslangic)
    print("FPS : ", fps)
    print("Karar : ",karar)

position(stm_pist[0],stm_pist[1],6,0)

while True :
    if (((vehicle.location.local_frame.north-stm_pist[0])**2+(vehicle.location.local_frame.east-stm_pist[1])**2)**0.5) < 0.5:
        break
    print(vehicle.location.local_frame.east,vehicle.location.local_frame.north)
while True :
    	ret,cv_image = cap.read()
    	cv_image = cv2.resize(cv_image,(640,480), interpolarization = cv2.INTER_AREA)
        ort_y,ort_x = islet (cv_image,thresh,dedet,isim,0,sift,train_descriptor[0],surf,bf)#STM icin detection
        vx,vy,aci,error= pid.update(ort_x,ort_y)
        print("HizX:{}, HizY:{}, Aci:{}, Error:{}".format(vx,vy,(aci*180)/math.pi,error))
        cv2.line(cv_image,(320,240),(ort_x,ort_y),(0,0,0),5)
        send_ned_velocity(vx,vy,0)
        if error<20:
            break

while True :
    ret,cv_image = cap.read()
    cv_image = cv2.resize(cv_image,(640,480), interpolarization = cv2.INTER_AREA)
    ort_y,ort_x = islet (cv_image,thresh,dedet,isim,0,sift,train_descriptor[0],surf,bf)#STM icin detection
    vehicle.mode='LAND'
    if vehicle.armed==0:
        break




