#!/usr/bin/env python
from __future__ import print_function
import sys
from pid_aci import PID
import cv2
import numpy as np
import time
from ip import baslat,islet, hangisi
from goruntu_isle1 import baslat,hangisi
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
import argparse

cap = cv2.imread(0)
train_descriptor=[]
score = [0,0,0,0,0,0,0,0,0,0]
alpha = 0

for each in range(5):
    thresh, dedet, isim, sift, train_descriptor_a, surf, bf = baslat(each)
    train_descriptor.append(train_descriptor_a)

while True :
    baslangic = time.time()
    ret,cv_image = cap.read()
    cv2.imwrite("kamil.jpg",cv_image)
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
