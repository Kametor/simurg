#!/usr/bin/env python
from __future__ import print_function
import sys
import cv2
import time
import numpy as np
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

from darknet_python import baslat, detect_et
from pid_aci import PID

pist_1=[4,4]
pist_2=[-4,4]
pist_3=[-4,-4]
pist_4=[4,-4]
pist_5=[0,0]


cap = cv2.imread(0)

pid = PID(0.001,0,0)
net,meta = baslat()

parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',               help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = '10.42.0.157:14550'
sitl = None

#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
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

arm_and_takeoff(6)
print("Set groundspeed to 3m/s.")
vehicle.groundspeed=3
print("Position North 4 East 4")

position(4, 4,6,0)

while True:
    if(3.5<vehicle.location.local_frame.east<4.5 and 3.5<vehicle.location.local_frame.north<4.5):
        break

while True :
    baslangic = time.time()
    ret,frame = cap.read()
    cv2.imwrite("kamil.jpg",frame)
    ort_x,ort_y,karar = detect_et(net,meta,"kamil.jpg")
    if karar != None :
        whichone = karar
        alpha = 0
        score = [0,0,0,0,0,0,0,0,0,0]
    if whichone=='stm':
        stm_pist = pist_1
        whichone='none'
        print('1.pistde stm')
        break
    elif whichone=='odtu':
        odtu_pist = pist_1
        whichone='none'
        print('1.pistde odtu')
        break
    elif whichone=='ort':
        ort_pist=pist_1
        whichone='none'
        print('1.pistde ort')
        break
    elif whichone=='landing':
        landing_pist=pist_1
        whichone='none'
        print('1.pistde landing')
        break
    bitis = time.time()
    fps=1/(bitis-baslangic)
    print("FPS : ", fps)
    print("Karar : ",karar)
    if karar != None:
        break

if karar =='stm':
    stm_pist = pist_1
    print('1.pistde stm')
elif karar=='odtu':
    odtu_pist = pist_1
    print('1.pistde odtu')
elif Karar=='ort':
    ort_pist=pist_1
    print('1.pistde ort')
elif karar =='landing':
    landing_pist=pist_1
    print('1.pistde landing')

position(4, -4, 6, 0)

while True :
    print('4 -4 gidiliyor')
    if(3.5<vehicle.location.local_frame.north<4.5 and -4.5<vehicle.location.local_frame.east<-3.5):
        break

while True :
    baslangic = time.time()
    ret,frame = cap.read()
    cv2.imwrite("kamil.jpg",frame)
    ort_x,ort_y,karar = detect_et(net,meta,"kamil.jpg")
    bitis = time.time()
    fps=1/(bitis-baslangic)
    print("FPS : ", fps)
    print("Karar : ",karar)
    if karar != None:
        break

if karar =='stm':
    stm_pist = pist_2
    print('2.pistde stm')
elif karar=='odtu':
    odtu_pist = pist_2
    print('2.pistde odtu')
elif Karar=='ort':
    ort_pist=pist_2
    print('2.pistde ort')
elif karar =='landing':
    landing_pist=pist_2
    print('2.pistde landing')

position(-4, -4, 6, 0)

while True :
    print('-4 -4 gidiliyor')
    if(-4.5<vehicle.location.local_frame.north<-3.5 and -4.5<vehicle.location.local_frame.east<-3.5):
        break

while True :
    baslangic = time.time()
    ret,frame = cap.read()
    cv2.imwrite("kamil.jpg",frame)
    ort_x,ort_y,karar = detect_et(net,meta,"kamil.jpg")
    bitis = time.time()
    fps=1/(bitis-baslangic)
    print("FPS : ", fps)
    print("Karar : ",karar)
    if karar != None:
        break

if karar =='stm':
    stm_pist = pist_3
    print('3.pistde stm')
elif karar=='odtu':
    odtu_pist = pist_3
    print('3.pistde odtu')
elif Karar=='ort':
    ort_pist=pist_3
    print('3.pistde ort')
elif karar =='landing':
    landing_pist=pist_3
    print('3.pistde landing')

position(-4, 4, 6, 0)

while True :
    print('-4 4 gidiliyor')
    if(-4.5<vehicle.location.local_frame.north<-3.5 and 3.5<vehicle.location.local_frame.east<4.5):
        break

while True :
    baslangic = time.time()
    ret,frame = cap.read()
    cv2.imwrite("kamil.jpg",frame)
    ort_x,ort_y,karar = detect_et(net,meta,"kamil.jpg")
    bitis = time.time()
    fps=1/(bitis-baslangic)
    print("FPS : ", fps)
    print("Karar : ",karar)
    if karar != None:
        break

if karar =='stm':
    stm_pist = pist_4
    print('4.pistde stm')
elif karar=='odtu':
    odtu_pist = pist_4
    print('4.pistde odtu')
elif Karar=='ort':
    ort_pist=pist_4
    print('4.pistde ort')
elif karar =='landing':
    landing_pist=pist_4
    print('4.pistde landing')

position(stm_pist[0],stm_pist[1],6,0)


while True :
    print('gorev 1')
    if((stm_pist[0] - 0.5)<vehicle.location.local_frame.north<(stm_pist[0]+0.5) and (stm_pist[1]-0.5)<vehicle.location.local_frame.east<(stm_pist+0.5)):
        break

while True :
    baslangic = time.time()
    ret,frame = cap.read()
    cv2.imwrite("kamil.jpg",frame)
    ort_x,ort_y,karar = detect_et(net,meta,"kamil.jpg")
    bitis = time.time()
    fps=1/(bitis-baslangic)
    print("FPS : ", fps)
    print("Karar : ",karar)
    print("tb_x:{}, tb_y:{}".format(ort_x,ort_y))
    vx,vy,aci,error= pid.update(ort_x,ort_y)
    print("HizX:{}, HizY:{}, Aci:{}, Error:{}".format(vx,vy,(aci*180)/math.pi,error))
    cv2.line(cv_image,(320,240),(ort_x,ort_y),(0,0,0),5)
    send_ned_velocity(vx,vy,0)
    if error<10:
        break

while True :
    vehicle.mode='LAND'
    print(vehicle.armed)
    if vehicle.armed==0:
        break

arm_and_takeoff(6)
