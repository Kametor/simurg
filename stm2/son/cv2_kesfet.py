#!/usr/bin/env python
from __future__ import print_function
import sys
from pid_aci import PID
import rospy
import cv2
import numpy as np
import time
from ip import baslat,islet, hangisi
from goruntu_isle1 import baslat,hangisi
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
import argparse

pist_1=[4,4]
pist_2=[-4,4]
pist_3=[-4,-4]
pist_4=[4,-4]
pist_5=[0,0]

cap = cv2.imread(0)
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

connection_string = '127.0.0.1:14550'
sitl = None

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

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
    karar, alpha, score =  hangisi(cv_image,thresh,dedet,isim,sift,train_descriptor,surf,bf,alpha,score)
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

position(4, -4, 6, 0)

while True :
    print('4 -4 gidiliyor')
    if(3.5<vehicle.location.local_frame.north<4.5 and -4.5<vehicle.location.local_frame.east<-3.5):
        break

while True :
    baslangic = time.time()
    ret,frame = cap.read()
    cv2.imwrite("kamil.jpg",frame)
    karar, alpha, score =  hangisi(cv_image,thresh,dedet,isim,sift,train_descriptor,surf,bf,alpha,score)
    if karar != None :
        whichone = karar
        alpha = 0
        score = [0,0,0,0,0,0,0,0,0,0]
    if whichone=='stm':
        stm_pist = pist_2
        whichone='none'
        print('2.pistde stm')
        break
    elif whichone=='odtu':
        odtu_pist = pist_2
        whichone='none'
        print('2.pistde odtu')
        break
    elif whichone=='ort':
        ort_pist=pist_2
        whichone='none'
        print('2.pistde ort')
        break
    elif whichone=='landing':
        landing_pist=pist_2
        whichone='none'
        print('2.pistde landing')
        break
    bitis = time.time()
    fps=1/(bitis-baslangic)
    print("FPS : ", fps)
    print("Karar : ",karar)

position(-4, -4, 6, 0)

while True :
    print('-4 -4 gidiliyor')
    if(-4.5<vehicle.location.local_frame.north<-3.5 and -4.5<vehicle.location.local_frame.east<-3.5):
        break

while True :
    baslangic = time.time()
    ret,frame = cap.read()
    cv2.imwrite("kamil.jpg",frame)
    karar, alpha, score =  hangisi(cv_image,thresh,dedet,isim,sift,train_descriptor,surf,bf,alpha,score)
    if karar != None :
        whichone = karar
        alpha = 0
        score = [0,0,0,0,0,0,0,0,0,0]
    if whichone=='stm':
        stm_pist = pist_3
        whichone='none'
        print('3.pistde stm')
        break
    elif whichone=='odtu':
        odtu_pist = pist_3
        whichone='none'
        print('3.pistde odtu')
        break
    elif whichone=='ort':
        ort_pist=pist_3
        whichone='none'
        print('3.pistde ort')
        break
    elif whichone=='landing':
        landing_pist=pist_3
        whichone='none'
        print('3.pistde landing')
        break
    bitis = time.time()
    fps=1/(bitis-baslangic)
    print("FPS : ", fps)
    print("Karar : ",karar)

position(-4, 4, 6, 0)

while True :
    print('-4 4 gidiliyor')
    if(-4.5<vehicle.location.local_frame.north<-3.5 and 3.5<vehicle.location.local_frame.east<4.5):
        break

while True :
    baslangic = time.time()
    ret,frame = cap.read()
    cv2.imwrite("kamil.jpg",frame)
    karar, alpha, score =  hangisi(cv_image,thresh,dedet,isim,sift,train_descriptor,surf,bf,alpha,score)
    if karar != None :
        whichone = karar
        alpha = 0
        score = [0,0,0,0,0,0,0,0,0,0]
    if whichone=='stm':
        stm_pist = pist_4
        whichone='none'
        print('4.pistde stm')
        break
    elif whichone=='odtu':
        odtu_pist = pist_4
        whichone='none'
        print('4.pistde odtu')
        break
    elif whichone=='ort':
        ort_pist=pist_4
        whichone='none'
        print('4.pistde ort')
        break
    elif whichone=='landing':
        landing_pist=pist_4
        whichone='none'
        print('4.pistde landing')
        break
    bitis = time.time()
    fps=1/(bitis-baslangic)
    print("FPS : ", fps)
    print("Karar : ",karar)

position(stm_pist[0],stm_pist[1],6,0)

while True :
    if (((vehicle.location.local_frame.north-self.stm_pist[0])**2+(vehicle.location.local_frame.east-self.stm_pist[1])**2)**0.5) < 0.5:
        break

while True :
        ort_y,ort_x = islet (cv_image,thresh,dedet,isim,0,sift,train_descriptor[0],surf,bf)#STM icin detection
        vx,vy,aci,error= self._pid.update(ort_x,ort_y)
            print("HizX:{}, HizY:{}, Aci:{}, Error:{}".format(vx,vy,(aci*180)/math.pi,error))
            cv2.line(cv_image,(320,240),(ort_x,ort_y),(0,0,0),5)
            send_ned_velocity(vx,vy,0)
            if error<10:
                break
While True :
    ort_y,ort_x = islet (cv_image,thresh,dedet,isim,0,sift,train_descriptor[0],surf,bf)#STM icin detection
    vehicle.mode='LAND'
    if vehicle.armed==0:
        break

arm_and_takeoff(6)
position(self.odtu_pist[0],self.odtu_pist[1],6,0)

while True:
    if ((((vehicle.location.local_frame.north+stm_pist[0])-odtu_pist[0])**2+((vehicle.location.local_frame.east+stm_pist[1])-odtu_pist[1])**2)**0.5) < 2.0:
        break

while True :
    ort_y,ort_x = islet (cv_image,thresh,dedet,isim,1,sift,train_descriptor[1],surf,bf)#STM icin detection
    vx,vy,aci,error= self._pid.update(ort_x,ort_y)
    print("HizX:{}, HizY:{}, Aci:{}, Error:{}".format(vx,vy,(aci*180)/math.pi,error))
    cv2.line(cv_image,(320,240),(ort_x,ort_y),(0,0,0),5)
    send_ned_velocity(vx,vy,0)
    if error<10:
        break

while True:
    ort_y,ort_x = islet (cv_image,thresh,dedet,isim,1,sift,train_descriptor[1],surf,bf)#STM icin detection
    vehicle.mode='LAND'
    if vehicle.armed==0:
        break

arm_and_takeoff(6)
position(self.ort_pist[0],self.ort_pist[1],6,0)

while True :
    if ((((vehicle.location.local_frame.north+self.odtu_pist[0])-self.ort_pist[0])**2+((vehicle.location.local_frame.east+self.odtu_pist[1])-self.ort_pist[1])**2)**0.5) < 2.0:
        break

while True :
    ort_y,ort_x = islet (cv_image,thresh,dedet,isim,2,sift,train_descriptor[2],surf,bf)#STM icin detection
    vx,vy,aci,error= self._pid.update(ort_x,ort_y)
    print("HizX:{}, HizY:{}, Aci:{}, Error:{}".format(vx,vy,(aci*180)/math.pi,error))
    cv2.line(cv_image,(320,240),(ort_x,ort_y),(0,0,0),5)
    send_ned_velocity(vx,vy,0)
    if error<10:
        break

while True :
    ort_y,ort_x = islet (cv_image,thresh,dedet,isim,2,sift,train_descriptor[2],surf,bf)#STM icin detection
    vehicle.mode='LAND'
    if vehicle.armed==0:
        break

arm_and_takeoff(6)
position(self.landing_pist[0],self.landing_pist[1],6,0)

while True :
        if ((((vehicle.location.local_frame.north+self.ort_pist[0])-self.landing_pist[0])**2+((vehicle.location.local_frame.east+self.ort_pist[1])-self.landing_pist[1])**2)**0.5) < 2.0:
        break

while True :
    ort_y,ort_x = islet (cv_image,thresh,dedet,isim,3,sift,train_descriptor[3],surf,bf)#STM icin detection
    vx,vy,aci,error= self._pid.update(ort_x,ort_y)
    print("HizX:{}, HizY:{}, Aci:{}, Error:{}".format(vx,vy,(aci*180)/math.pi,error))
    cv2.line(cv_image,(320,240),(ort_x,ort_y),(0,0,0),5)
    send_ned_velocity(vx,vy,0)
    if error<10:
        break

while True :
    ort_y,ort_x = islet (cv_image,thresh,dedet,isim,3,sift,train_descriptor[3],surf,bf)#STM icin detection
    vehicle.mode='LAND'
    if vehicle.armed==0:
        break
