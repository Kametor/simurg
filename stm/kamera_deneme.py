import cv2

def gstreamer_pipeline() :   
	return ('nvarguscamerasrc ! ' 
	'video/x-raw(memory:NVMM), '
	'width=(int)1280, height=(int)720, '
	'format=(string)NV12, framerate=(fraction)24/1 ! '
	'nvvidconv flip-method=2 ! '
	'video/x-raw, width=640, height=(int)480, format=(string)BGRx ! '
	'videoconvert ! '
	'video/x-raw, format=(string)BGR ! appsink')

#kamera = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
kamera = cv2.VideoCapture(0)


while True :
	print("kamera acik")
	grabbed, frame = kamera.read()
	if not grabbed :
		break
	cv2.imshow("test", frame)
	key = cv2.waitKey(1)
	# q ya basilirsa kapanir
	if key & 0xFF == ord("q") :
		cv2.imwrite("asd.jpg",frame)
