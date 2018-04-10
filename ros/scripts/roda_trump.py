import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5cl

check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

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

def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro = identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)



if __name__=="__main__":

	rospy.init_node("trump")
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
	#recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame)
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media) != 0 and len(centro) != 0:
				dif_x = media[0]-centro[0]
				dif_y = media[1]-centro[1]
				if math.fabs(dif_x)<30: # Se a media estiver muito proxima do centro anda para frente
					vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
				else:
					if dif_x > 0: # Vira a direita
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
					else: # Vira a esquerda
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
