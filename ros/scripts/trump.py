import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

# Parameters to use when opening the webcam.
cap = cv2.VideoCapture(0)
if cap.isOpened():
	print("Webcam aberta")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

trump = cv2.imread('/home/borg/catkin_ws/src/Projeto1_RC/ros/python/scripts/trump.png')

# Imagem a procurar
img1 = cv2.cvtColor(trump, cv2.COLOR_RGB2GRAY)

# Initiate SIFT detector
sift = cv2.xfeatures2d.SIFT_create()

# find the keypoints and descriptors with SIFT in each image
kp1, des1 = sift.detectAndCompute(img1,None)


while(True):
	# Capture frame-by-frame
	print("New frame")
	ret, frame = cap.read()

	#if ret == False:
	#	continue
	#if frame is None:
		#print("Frame e'  nulo")

	cv2.imshow("Imagem lida", frame)
	# Convert the frame to grayscale
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	# A gaussian blur to get rid of the noise in the image
	video = cv2.GaussianBlur(gray,(5,5),0)
	# Detect the edges present in the image
	bordas = cv2.Canny(video, 50, 200)
	#agora comparar oq volta do matches
	bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)

	
	#MAD FOX
	
	MIN_MATCH_COUNT =100
	#while True:
		#if cv2.waitKey(1) & 0xFF == ord('q'):
			#break
	img2 = video # Imagem do cenario - puxe do video para fazer isto
	kp2, des2 = sift.detectAndCompute(img2,None)


	FLANN_INDEX_KDTREE = 0
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks = 50)

	# Configura o algoritmo de casamento de features
	flann = cv2.FlannBasedMatcher(index_params, search_params)

	# Tenta fazer a melhor comparacao usando o algoritmo
	#Oq volta desse matches
	matches = flann.knnMatch(des1,des2,k=2)

	# store all the good matches as per Lowe's ratio test.
	good = []
	for m,n in matches:
		if m.distance < 0.7*n.distance:
				good.append(m)
	
	#CIRCULO
	
	circles = []
	# Obtains a version of the edges image where we can draw in color
	bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)
	circles = None
	circles=cv2.HoughCircles(bordas,cv2.HOUGH_GRADIENT,2,40,param1=50,param2=100,minRadius=5,maxRadius=60)


	if len(good)>MIN_MATCH_COUNT and circles is not None:
		

		# Separa os bons matches na origem e no destino
		src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)


		# Tenta achar uma trasformacao composta de rotacao, translacao e escala que situe uma imagem na outra
		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
		matchesMask = mask.ravel().tolist()

		#tentativa de "printar" keyponits continuacao
	

		#menorx=0
		#menory=0
		#maiorx=1000
		#maiory=1000
		#for i in x:
			#if i<menorx:
				#menorx=i
			#if i>maiorx:
				#maiorx=x
		#for i in y:
			#if i<menory:
				#menory=i
			#if i>maiory:
				#maiory=x
		# cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]])
		cv2.rectangle(bordas_color,(384,0),(510,128),(0,255,0),3)

		circles = np.uint16(np.around(circles))

		for i in circles[0,:]:
			print(i)
			# draw the outer circle
			# cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
			cv2.circle(bordas_color,(i[0],i[1]),i[2],(0,255,0),2)
			# draw the center of the circle
			cv2.circle(bordas_color,(i[0],i[1]),2,(0,0,255),3)


		font = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(bordas_color,'Achou trump',(0,50), font, 2,(255,255,255),2,cv2.LINE_AA)



	cv2.imshow('Detector de trump',bordas_color)
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cap.release()