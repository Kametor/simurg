import cv2
from goruntu_isle1 import hangisi,baslat
import time

cap = cv2.VideoCapture("video1.mp4")
train_descriptor=[]
score = [0,0,0,0,0,0,0,0,0,0]
alpha = 0
for each in range(5):
    thresh, dedet, isim, sift, train_descriptor_a, surf, bf = baslat(each)
    train_descriptor.append(train_descriptor_a)

while True :
    baslangic = time.time()
    ret,cv_image = cap.read()
    print(ret)
    karar, alpha, score =  hangisi(cv_image,thresh,dedet,isim,sift,train_descriptor,surf,bf,alpha,score)
    print(karar)
    bitis = time.time()
    print(1.0/(bitis-baslangic))
