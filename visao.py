#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from std_msgs.msg import UInt8
from cv_bridge import CvBridge, CvBridgeError
import cormodule
from math import pi
import tf
import visao_module


batida=0
frontal=0
v=0.1
w =(pi/2)
w2 =0.3
bridge = CvBridge()
cv_image = None
mediaA = []
centroA = []
atraso = 0.5E9 # 1 segundo e meio. Em nanossegundos
areaV = 0.0 # Variavel com a area do maior contorno
areaA = 0.0
metade=320
sigma=5
media = []
centro = []
Pi=None
Pf=None
posicao=None
viu_dog = False
media_dog=None
check_delay = False

def bateu(dado):
	global batida
	batida = dado.data

def scaneou(dado):
	global frontal
	frontal = dado.ranges[0]

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	#print("frame")
	global cv_image
	global mediaA
	global centroA
	global areaA
	global media
	global centro
	global viu_dog
	global posicao
	global Pi
	global Pf
	global media_dog

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	#print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		#mediaV, centroV, areaV =  cormodule.identifica_cor_vermelho(cv_image)
		mediaA, centroA, areaA =  cormodule.identifica_cor_azul(cv_image)
		centro, imagem, resultados =  visao_module.processa(cv_image)
        #vc_temp = find_circles(cv_image)
        # if vc_temp:
        #     viu_circulo = True

		for r in resultados:
		# print(r) - print feito para documentar e entender
			if r[0] == "cat":
				viu_dog = True
				Pi=r[2]
				Pf=r[3]
				posicao=(Pi[0],Pi[1],Pf[0],Pf[1])
				media_dog=(posicao[0]+((posicao[2]-posicao[0]))/2)
				#print(r)

		depois = time.clock()

	except CvBridgeError as e:
		print('ex', e)
	
if __name__=="__main__":
	rospy.init_node("cor")

	topico_imagem = "/kamera"
	recebe_Bumper = rospy.Subscriber("/bumper", UInt8, bateu)
	
	# Para renomear a *webcam*
	# 
	# 	rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
	# 
	# Para renomear a câmera simulada do Gazebo
	# 
	# 	rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
	# 
	# Para renomear a câmera da Raspberry
	# 
	# 	rosrun topic_tools relay /raspicam_node/image/compressed /kamera
	# 

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	#print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():

			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			
			#BUMPERS - Survival
			if batida != 0:

				if batida == 2:

					print("bati 2 vou recalcular rota")
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)

					vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					
					vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					
					vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(2.0)
					
					vel = Twist(Vector3(0,0,0), Vector3(0,0,-w))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)

					batida=0

				elif batida == 1:

					print("bati 1 vou recalcular rota")
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)

					vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					
					vel = Twist(Vector3(0,0,0), Vector3(0,0,-w))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					
					vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(2.0)
					
					vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)

					batida=0

				elif batida == 3:

					print("bati 3 vou recalcular rota")
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)

					vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					
					vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					
					vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					
					vel = Twist(Vector3(0,0,0), Vector3(0,0,-w))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)

					vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(4.0)

					batida=0

				elif batida == 4:

					print("bati 4 vou recalcular rota")
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)

					vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					
					vel = Twist(Vector3(0,0,0), Vector3(0,0,-w))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					
					vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(4.0)
					
					vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)

					vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(4.0)

					batida=0

			# DETECTOR DE PROXIMIDADE LASERSCAN - SURVIVAL
				#if min da lista de todos os angulos for mto prox regir





			# DETECTOR MOBILENET - Friendly

				if posicao != None:

					print(" a media da posicao do dog eh{0}".format(media_dog))

					if media_dog<metade-sigma:
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-w2))
						velocidade_saida.publish(vel)
						rospy.sleep(0.8)
						viu_dog=False
						print("seguindo dog indo pra esquerda")
						
					elif media_dog>metade+sigma:
						vel = Twist(Vector3(0,0,0), Vector3(0,0,w2))
						velocidade_saida.publish(vel)
						rospy.sleep(0.8)
						viu_dog = False
						print("seguindo dog indo pra direita")
						
					else:
						vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1)
						viu_dog = False
						print("seguindo dog")

			# DETECTOR DE COR - Friendly

			elif areaA>5000:

				if mediaA[0] < metade - sigma:
					vel = Twist(Vector3(-v,0,0), Vector3(0,0,-w2))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					print("fugindo do azul desviando para direita")
				
				elif mediaA[0] > metade + sigma:
					vel = Twist(Vector3(-v,0,0), Vector3(0,0,w2))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					print("fugindo do azul desviando para esquerda")
				
				else:
					vel = Twist(Vector3(-v,0,0), Vector3(0,0,0))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					print("fugindo do azul")
				
			else:

				vel = Twist(Vector3(0,0,0), Vector3(0,0,w2))
				velocidade_saida.publish(vel)
				rospy.sleep(0.1)
				print("Procurando a acao")

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


