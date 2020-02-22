import os
import sys
sys.path.append('/home/yusuf/darknet/python/')
import darknet as dn
import pdb
import cv2
import numpy as np



def baslat ():
  dn.set_gpu(0)
  net = dn.load_net("/home/yusuf/darknet/cfg/yolov3-tiny.cfg", "/home/yusuf/darknet/yolov3-tiny.weights", 0)
  meta = dn.load_meta("/home/yusuf/darknet/cfg/coco.data")
  return net,meta

def detect_et (net,meta,frame):
  r = dn.detect(net, meta, frame)
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

frame = "kamil.jpg"
net,meta =  baslat()
detect_et(net,meta,frame)
