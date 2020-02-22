#!/usr/bin/env python
from __future__ import print_function
import sys
from pid_aci import PID
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import time
from ip import baslat,islet, hangisi
from goruntu_isle1 import baslat,hangisi
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
import argparse

a=5
gorev=1.1
thresh, dedet, isim, sift, train_descriptor, surf, bf = baslat(0)
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
connection_string = '127.0.0.1:14550'
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


    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

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
        if remainingDistance<=targetDistance*0.8: #Just below target, in case of undershoot.
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
print("Set groundspeed to 5m/s.")
vehicle.groundspeed=3
print("Position North 4 East 4")
position(4,4,6,0)

class image_converter:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/drone/camera1/image_raw",Image,self.callback)
    self.train_descriptor=[]
    self.score = [0,0,0,0,0,0,0,0,0,0]
    self.alpha = 0
    self._pid = PID(0.001,0,0)
    self.whichone = 'none'
    self.gorev = 1.11
    self.pist_1=[4,4]
    self.pist_2=[-4,4]
    self.pist_3=[-4,-4]
    self.pist_4=[4,-4]
    self.pist_5=[0,0]
    self.stm_pist = None
    self.odtu_pist=None
    self.ort_pist=None
    self.landing_pist=None
    for each in range(5):
        self.thresh, self.dedet, self.isim, self.sift, train_descriptor_a, self.surf, self.bf = baslat(each)
        self.train_descriptor.append(train_descriptor_a)
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.imshow("kamera_ekrani",cv_image)
      cv2.waitKey(3)
      height, width = 320,240
    except CvBridgeError as e:
      print(e)

# Gorev kodu basliyor...
    if self.gorev==1.11:
        print("gorev:1.11")
        if(3.5<vehicle.location.local_frame.east<4.5 and 3.5<vehicle.location.local_frame.north<4.5):
            self.gorev=1.12
    elif self.gorev==1.12:
        print("gorev:1.12")
        karar, self.alpha, self.score =  hangisi(cv_image,self.thresh,self.dedet,self.isim,self.sift,self.train_descriptor,self.surf,self.bf,self.alpha,self.score)
        print(karar)
        if karar != None :
            self.whichone = karar
            self.alpha = 0
            self.score = [0,0,0,0,0,0,0,0,0,0]
        if self.whichone=='stm':
            self.stm_pist = self.pist_1
            self.gorev=1.21
            self.whichone='none'
            print('1.pistde stm')
        elif self.whichone=='odtu':
            self.odtu_pist = self.pist_1
            self.gorev=1.21
            self.whichone='none'
            print('1.pistde odtu')
        elif self.whichone=='ort':
            self.ort_pist=self.pist_1
            self.gorev=1.21
            self.whichone='none'
            print('1.pistde ort')
        elif self.whichone=='landing':
            self.landing_pist=self.pist_1
            self.self.gorev=1.21
            self.whichone='none'
            print('1.pistde landing')
    elif self.gorev==1.21:
        print("gorev:1.21")
        print('-4 4 gidiliyor')
        position(-4, 4, 6, 0)
        self.gorev=1.22
    elif self.gorev==1.22:
        print("gorev:1.22")
        if(-4.5<vehicle.location.local_frame.north<-3.5 and 3.5<vehicle.location.local_frame.east<4.5):
            self.gorev=1.23
    elif self.gorev==1.23:
        print("gorev:1.23")
        karar, self.alpha, self.score =  hangisi(cv_image,self.thresh,self.dedet,self.isim,self.sift,self.train_descriptor,self.surf,self.bf,self.alpha,self.score)
        print(karar)
        if karar != None :
            self.whichone = karar
            self.alpha = 0
            self.score = [0,0,0,0,0,0,0,0,0,0]
        if self.whichone=='stm':
            self.stm_pist = self.pist_2
            self.gorev=1.31
            self.whichone='none'
            print('2.pistde stm')
        elif self.whichone=='odtu':
            self.odtu_pist = self.pist_2
            self.gorev=1.31
            self.whichone='none'
            print('2.pistde odtu')
        elif self.whichone=='ort':
            self.ort_pist=self.pist_2
            self.gorev=1.31
            self.whichone='none'
            print('2.pistde ort')
        elif self.whichone=='landing':
            self.landing_pist=self.pist_2
            self.gorev=1.31
            self.whichone='none'
            print('2.pistde landing')
    elif self.gorev==1.31:
        print("gorev:1.31")
        position(-4, -4, 6, 0)
        self.gorev=1.32
    elif self.gorev==1.32:
        print("gorev:1.32")
        if(-4.5<vehicle.location.local_frame.north<-3.5 and -4.5<vehicle.location.local_frame.east<-3.5):
            self.gorev=1.33
    elif self.gorev==1.33:
        print("gorev:1.33")
        karar, self.alpha, self.score =  hangisi(cv_image,self.thresh,self.dedet,self.isim,self.sift,self.train_descriptor,self.surf,self.bf,self.alpha,self.score)
        print(karar)
        if karar != None :
            self.whichone = karar
            self.alpha = 0
            self.score = [0,0,0,0,0,0,0,0,0,0]
        if self.whichone=='stm':
            self.stm_pist = self.pist_3
            self.gorev=1.41
            self.whichone='none'
            print('3.pistde stm')
        elif self.whichone=='odtu':
            self.odtu_pist = self.pist_3
            self.gorev=1.41
            self.whichone='none'
            print('3.pistde odtu')
        elif self.whichone=='ort':
            self.ort_pist=self.pist_3
            self.gorev=1.41
            self.whichone='none'
            print('3.pistde ort')
        elif self.whichone=='landing':
            self.landing_pist=self.pist_3
            self.gorev=1.41
            self.whichone='none'
            print('3.pistde landing')
    elif self.gorev==1.41:
        print("gorev:1.41")
        position(4, -4, 6, 0)
        self.gorev=1.42
    elif self.gorev==1.42:
        print("gorev:1.42")
        if(3.5<vehicle.location.local_frame.north<4.5 and -4.5<vehicle.location.local_frame.east<-3.5):
            self.gorev=1.43
    elif self.gorev==1.43:
        karar, self.alpha, self.score =  hangisi(cv_image,self.thresh,self.dedet,self.isim,self.sift,self.train_descriptor,self.surf,self.bf,self.alpha,self.score)
        print(karar)
        if karar != None :
            self.whichone = karar
            self.alpha = 0
            self.score = [0,0,0,0,0,0,0,0,0,0]
        if self.whichone=='stm':
            self.stm_pist = self.pist_4
            self.gorev=2.11
            self.whichone='none'
            print('4.pistde stm')
        elif self.whichone=='odtu':
            self.odtu_pist = self.pist_4
            self.gorev=2.11
            self.whichone='none'
            print('4.pistde odtu')
        elif self.whichone=='ort':
            self.ort_pist=self.pist_4
            self.gorev=2.11
            self.whichone='none'
            print('4.pistde ort')
        elif self.whichone=='landing':
            self.landing_pist=self.pist_4
            self.gorev=2.11
            self.whichone='none'
            print('4.pistde landing')
    elif self.gorev==2.11:
        print("gorev:2.11")
        position(self.stm_pist[0], self.stm_pist[1], 6, 0)
        self.gorev=2.12
    elif self.gorev==2.12:
        print("Gorev 2.12")
        print(vehicle.location.local_frame.north)
        print(vehicle.location.local_frame.east)
        print(((vehicle.location.local_frame.north-self.stm_pist[0])**2+(vehicle.location.local_frame.east-self.stm_pist[1])**2)**0.5)
        if (((vehicle.location.local_frame.north-self.stm_pist[0])**2+(vehicle.location.local_frame.east-self.stm_pist[1])**2)**0.5) < 0.5:
            self.gorev=2.13

    elif self.gorev==2.13:
        print("Gorev 2.13")
        ort_y,ort_x = islet (cv_image,thresh,dedet,isim,0,sift,train_descriptor,surf,bf)#STM icin detection
        print("tb_x:{}, tb_y:{}".format(ort_x,ort_y))
        vx,vy,aci,error= self._pid.update(ort_x,ort_y)
        print("HizX:{}, HizY:{}, Aci:{}, Error:{}".format(vx,vy,(aci*180)/math.pi,error))
        cv2.line(cv_image,(320,240),(ort_x,ort_y),(0,0,0),5)
        send_ned_velocity(vx,vy,0)
        if error<10:
            self.gorev=2.14

    elif self.gorev==2.14:
        print("Gorev 2.14")
        ort_y,ort_x = islet (cv_image,thresh,dedet,isim,0,sift,train_descriptor,surf,bf)#STM icin detection
	print(ort_x,ort_y)
        vehicle.mode='LAND'
        print(vehicle.armed)
        if vehicle.armed==0:
            self.gorev=2.21

    elif self.gorev==2.21:
        print("Gorev 2.21")
        arm_and_takeoff(6)
        position(self.odtu_pist[0],self.odtu_pist[1],6,0)
        self.gorev=2.22
    elif self.gorev==2.22:
        print("Gorev 2.22")
        print((vehicle.location.local_frame.north)+self.stm_pist[0])
        print((vehicle.location.local_frame.east)+self.stm_pist[1])
        print(((vehicle.location.local_frame.north)**2+(vehicle.location.local_frame.east)**2)**0.5)
        if ((((vehicle.location.local_frame.north+self.stm_pist[0])-self.odtu_pist[0])**2+((vehicle.location.local_frame.east+self.stm_pist[1])-self.odtu_pist[1])**2)**0.5) < 2.0:
            self.gorev=2.23

    elif self.gorev==2.23:
        print("Gorev 2.23")
        ort_y,ort_x = islet (cv_image,thresh,dedet,isim,1,sift,self.train_descriptor[1],surf,bf)#odtu icin detection
        print("tb_x:{}, tb_y:{}".format(ort_x,ort_y))
        vx,vy,aci,error= self._pid.update(ort_x,ort_y)
        print("HizX:{}, HizY:{}, Aci:{}, Error:{}".format(vx,vy,(aci*180)/math.pi,error))
        cv2.line(cv_image,(320,240),(ort_x,ort_y),(0,0,0),5)
        send_ned_velocity(vx,vy,0)
        if error<10:
            self.gorev=2.24

    elif self.gorev==2.24:
        print("Gorev 2.24")
        ort_y,ort_x = islet (cv_image,thresh,dedet,isim,1,sift,self.train_descriptor[1],surf,bf)#odtu icin detection
        vehicle.mode='LAND'
        print(vehicle.armed)
        if vehicle.armed==0:
            self.gorev=2.31
    elif self.gorev==2.31:
        print("Gorev 2.31")
        arm_and_takeoff(6)
        position(self.ort_pist[0],self.ort_pist[1],6,0)
        self.gorev=2.32
    elif self.gorev==2.32:
        print("Gorev 2.32")
        print((vehicle.location.local_frame.north)+self.odtu_pist[0])
        print((vehicle.location.local_frame.east)+self.odtu_pist[1])
        print(((vehicle.location.local_frame.north-self.ort_pist[0])**2+(vehicle.location.local_frame.east-self.ort_pist[1])**2)**0.5)
        if ((((vehicle.location.local_frame.north+self.odtu_pist[0])-self.ort_pist[0])**2+((vehicle.location.local_frame.east+self.odtu_pist[1])-self.ort_pist[1])**2)**0.5) < 2.0:
            self.gorev=2.33
    elif self.gorev==2.33:
        print("Gorev 2.33")
        ort_y,ort_x = islet (cv_image,thresh,dedet,isim,2,sift,self.train_descriptor[2],surf,bf)#ort icin detection
        print("tb_x:{}, tb_y:{}".format(ort_x,ort_y))
        vx,vy,aci,error= self._pid.update(ort_x,ort_y)
        print("HizX:{}, HizY:{}, Aci:{}, Error:{}".format(vx,vy,(aci*180)/math.pi,error))
        cv2.line(cv_image,(320,240),(ort_x,ort_y),(0,0,0),5)
        send_ned_velocity(vx,vy,0)
        if error<10:
            self.gorev=2.34
    elif self.gorev==2.34:
        print("Gorev 2.34")
        ort_y,ort_x = islet (cv_image,thresh,dedet,isim,2,sift,self.train_descriptor[2],surf,bf)#ort icin detection
        vehicle.mode='LAND'
        print(vehicle.armed)
        if vehicle.armed==0:
            self.gorev=2.41

    elif self.gorev==2.41:
        print("Gorev 2.41")
        arm_and_takeoff(6)
        position(self.landing_pist[0],self.landing_pist[1],6,0)
        self.gorev=2.42
    elif self.gorev==2.42:
        print("Gorev 2.42")
        print((vehicle.location.local_frame.north)+self.ort_pist[0])
        print((vehicle.location.local_frame.east)+self.ort_pist[1])
        print(((vehicle.location.local_frame.north-self.landing_pist[0])**2+(vehicle.location.local_frame.east-self.landing_pist[1])**2)**0.5)
        if ((((vehicle.location.local_frame.north+self.ort_pist[0])-self.landing_pist[0])**2+((vehicle.location.local_frame.east+self.ort_pist[1])-self.landing_pist[1])**2)**0.5) < 2.0:
            self.gorev=2.43

    elif self.gorev==2.43:
        print("Gorev 2.43")
        ort_y,ort_x = islet (cv_image,thresh,dedet,isim,3,sift,self.train_descriptor[3],surf,bf)#landing icin detection
        print("tb_x:{}, tb_y:{}".format(ort_x,ort_y))
        vx,vy,aci,error= self._pid.update(ort_x,ort_y)
        print("HizX:{}, HizY:{}, Aci:{}, Error:{}".format(vx,vy,(aci*180)/math.pi,error))
        cv2.line(cv_image,(320,240),(ort_x,ort_y),(0,0,0),5)
        send_ned_velocity(vx,vy,0)
        if error<10:
            self.gorev=2.44

    elif self.gorev==2.44:
        print("Gorev 2.44")
        ort_y,ort_x = islet (cv_image,thresh,dedet,isim,3,sift,self.train_descriptor[3],surf,bf)#landing icin detection
        vehicle.mode='LAND'
        print(vehicle.armed)
        if vehicle.armed==0:
            self.gorev=3
    elif self.gorev==3:
        print("Gorev 3")
        print("Gorev Basarili...")

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
