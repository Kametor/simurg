import cv2
import time
def baslat (sira) :
    thresh=[200,0.25,200,150,150]
    isim=["stm","odtu","ort","landing","TURKBAYRAGI"]
    dedet=[15,7,5,7,3]


    image1 = cv2.imread('pictures/'+str(sira)+'.png')
    training_image = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)
    training_gray = cv2.cvtColor(training_image, cv2.COLOR_RGB2GRAY)
    sift = cv2.xfeatures2d.SIFT_create()
    surf = cv2.xfeatures2d.SURF_create(800)
    if(sira!=1):
        train_keypoints, train_descriptor = sift.detectAndCompute(training_gray, None)
    else:
        train_keypoints, train_descriptor = surf.detectAndCompute(training_gray, None)
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck = False)

    return thresh,dedet,isim,sift,train_descriptor,surf,bf

def isle (cv_image,thresh,dedet,isim,sift,train_descriptor,surf,bf) :
    score_depo = []
    for sira in range(5):

        # Capture frame-by-frame
        test_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Algoritma cagirma
        if(sira!=1):
            test_keypoints, test_descriptor = sift.detectAndCompute(test_gray, None)
        else:
            test_keypoints, test_descriptor = surf.detectAndCompute(test_gray, None)

        # BF MATCH
        matches = bf.match(train_descriptor[sira], test_descriptor)
        # APPLY THRESHOLD
	thres = thresh[sira]

	i = 0
	liste1 = []
	for each in matches:
            if float(each.distance) > thres:
                liste1.append(i)
	    i += 1

	l=0
	for k in liste1:
	    del matches[k-l]
	    l += 1
        if len(matches) > dedet[sira] :
            score = float(len(matches))/float(len(train_descriptor[sira]))
            score_depo.append(score)

        else :
            score_depo.append(0)

    return score_depo

def hangisi(cv_image,thresh,dedet,isim,sift,train_descriptor,surf,bf,alpha,score):
    if alpha != 10 :

        score[alpha] = isle(cv_image,thresh,dedet,isim,sift,train_descriptor,surf,bf)
        alpha += 1
        karar = None
    else :
        sonuc = [0,0,0,0,0]
        for beta in range(10):
            for gama in range(5):
                sonuc[gama] += score[beta][gama]
        maximum_olan = 0
        print(sonuc)
        for indeks in range(5):
            if(sonuc[indeks]>sonuc[maximum_olan]):
                maximum_olan = sonuc[indeks]
        karar = isim[maximum_olan]
    return karar,alpha,score

def islet (cv_image,thresh,dedet,isim,sira,sift,train_descriptor,surf,bf) :
    baslama_zamani = time.time()
    # Capture frame-by-frame
    test_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Algoritma cagirma
    if(sira!=1):
        test_keypoints, test_descriptor = sift.detectAndCompute(test_gray, None)
    else:
        test_keypoints, test_descriptor = surf.detectAndCompute(test_gray, None)

    # BF MATCH
    matches = bf.match(train_descriptor, test_descriptor)

    # APPLY THRESHOLD
    thres = thresh[sira]
    i = 0
    liste1 = []
    for each in matches:
        if float(each.distance) > thres:
                liste1.append(i)
        i += 1

    l=0
    for k in liste1:
        del matches[k-l]
        l += 1

    # GET PIXEL LOCATION
    list_kp1 = []

    for mat in matches:

        # Get the matching keypoints for each of the images
        img2_idx = mat.trainIdx

        # x - columns
        # y - rows
        # Get the coordinates

        (x2,y2) = test_keypoints[img2_idx].pt
        list_kp1.append((x2, y2))

    totala = 0
    totalb = 0
    index = 1

        # GET AVERAGE PIXEL VALUE FOR X AND Y AXIS
    for every in list_kp1:
        a = int(every[1])
        b = int(every[0])
        totala += a
        totalb += b
        # test_gray[a-1:a+2,b-1:b+2] = 0
        index += 1

    ort_a = int((totala)/(index))
    ort_b = int((totalb)/(index))

        # STORE THE AVERAGE PIXEL IN A LIST.

        # GET THE PIXEL LOCATION AVERAGING PREVIOUS PIXEL VALUES

        # DECIDE DETECTION
    if len(matches) > dedet[sira] :
        test_gray[ort_a-3:ort_a+4,ort_b-3:ort_b+4] = 0
    bitis_zamani=time.time()
    fps=1/(bitis_zamani-baslama_zamani)
       # Display the best matching points


        # Print total number of matching points between the training and query images
    print("\nNumber of Matching Keypoints Between The Training and Query Images ( "+isim[sira]+" ): ", len(matches))
    print("FPS:",str(fps))
    cv2.imshow("asd",test_gray)
    cv2.waitKey(3)
    return ort_a,ort_b
