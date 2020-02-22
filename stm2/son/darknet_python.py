import os
import sys
sys.path.append('/home/yusuf/darknet/python/')
import darknet as dn
import pdb
import cv2
import numpy as np
import time



def baslat ():
  dn.set_gpu(0)
  net = dn.load_net("/home/yusuf/darknet/cfg/yolov3-tiny.cfg", "/home/yusuf/darknet/yolov3-tiny.weights", 0)
  meta = dn.load_meta("/home/yusuf/darknet/cfg/coco.data")
  return net,meta

def detect_et (net,meta,frame):
  r = dn.detect(net, meta, frame)
  ort_x = None
  ort_y = None
  karar = None
  if len(r) != 0:
      max = r[0][1]
      i = 0
      k = 0

      karar = r[0][0]
      ort_x_min = r[0][2][0]
      ort_y_min = r[0][2][1]
      ort_w = r[0][2][2]
      ort_h = r[0][2][3]

      ort_x = int(ort_x_min + (ort_w/2.0))
      ort_y = int(ort_y_min + (ort_h/2.0))

  return ort_x,ort_y,karar

cap = cv2.VideoCapture(0)
net,meta =  baslat()

while True :
    baslangic = time.time()
    ret,frame = cap.read()
    cv2.imwrite("kamil.jpg",frame)
    ort_x,ort_y,karar = detect_et(net,meta,"kamil.jpg")
    bitis = time.time()
    fps=1/(bitis-baslangic)
    print("FPS : ", fps)
    print("Karar : ",karar)
