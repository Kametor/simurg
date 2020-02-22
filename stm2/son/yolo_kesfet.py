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
from utils import arm_and_takeoff, goto, get_location_metres, get_distance_metres, send_ned_velocity, position
from pid_aci import PID

pist_1=[4,4]
pist_2=[-4,4]
pist_3=[-4,-4]
pist_4=[4,-4]
pist_5=[0,0]

a=5

cap = cv2.imread(0)

pid = PID(0.001,0,0)
net,meta = baslat()

parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',               help="Vehicle connection target string. If not specified, SITL automatically started and used.")
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
    karar, alpha, score =  hangisi(cv_image,self.thresh,self.dedet,self.isim,self.sift,self.train_descriptor,self.surf,self.bf,self.alpha,self.score)
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
