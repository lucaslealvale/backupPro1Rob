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
#import visao_module


batida=0
#frontal=0
v=0.1
w =(pi/2)

def bateu(dado):
	global batida
	batida = dado.data

print("ta rodando o certo")
bridge = CvBridge()

cv_image = None
#mediaV = []
#centroV = []

mediaA = []
centroA = []

atraso = 0.5E9 # 1 segundo e meio. Em nanossegundos

areaV = 0.0 # Variavel com a area do maior contorno
areaA = 0.0
metade=320
sigma=25
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
media = []
centro = []
Pi=None
Pf=None
posicao=None
viu_dog = False
media_dog=None
# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	#print("frame")
	global cv_image
	global mediaA
	#global mediaV
	global centroA
	#global centroV
	global areaV
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
		# o resultado
			if r[0] == "dog":
				viu_dog = True
				Pi=r[2]
				Pf=r[3]
				posicao=(Pi[0],Pi[1],Pf[0],Pf[1])
				media_dog=(posicao[2]-posicao[0])/2
				#print("resultados")
		depois = time.clock()
		cv2.imshow("Camera", cv_image)

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
			
			if batida != 0:
				if batida == 2:
					print("bati vou recalcular rota")
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
					print("bati vou recalcular rota")
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
					print("bati vou recalcular rota")
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
					print("bati vou recalcular rota")
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

			if viu_dog:
                #posicao xi, yi , xf, yf so me importa x
				if posicao != None:
					
					#media_dog=(posicao[2]-posicao[0])/2
					print(" a media da posicao do dog eh{0}".format(media_dog))

					if media_dog<metade+sigma:
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.3))
						velocidade_saida.publish(vel)
						rospy.sleep(0.8)
						print("esquerda para ir ao dog")

					elif media_dog>metade+sigma:
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.3))
						velocidade_saida.publish(vel)
						rospy.sleep(0.8)
						viu_cat = False
						print("direita pra ir ao dog")

					else:
						vel = Twist(Vector3(0.4,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(0.8)
						viu_cat = False
						print("Segue Dog")
						continue

                # vel = Twist(Vector3(0.4,0,0), Vector3(0,0,0))
                # velocidade_saida.publish(vel)
                # rospy.sleep(0.8)
                # viu_cat = False
                # continue
			else:
				if (len(mediaA) != 0 and len(centroA) != 0):# and len(mediaA) !=0  and len(centroA) !=0):
					
				#if areaA != 0 or areaV != 0:
					#print("passei")
					#if areaV > areaA:
						
					# 	if mediaV[0] < metade + sigma:
					# 		vel = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.3))
					# 		#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					# 		velocidade_saida.publish(vel)
					# 		rospy.sleep(1.0)
					# 		print("direitaa vermelho")
						
					# 	elif mediaV[0] > metade + sigma:
					# 		vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.3))
					# 		#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					# 		velocidade_saida.publish(vel)
					# 		rospy.sleep(1.0)
					# 		print("esquerdaa vermelho")
						
					# 	else:
					# 		vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
					# 		#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
					# 		velocidade_saida.publish(vel)
					# 		rospy.sleep(1.0)
					# 		print("em frente pro vermelho!")
					
					# elif areaV < areaA:
						
					if mediaA[0] < metade + sigma:
						vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,-0.3))
						#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						print("direitaa foge do azul")
					
					elif mediaA[0] > metade + sigma:
						vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,0.3))
						#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						print("esquerdaa run do azul")
					
					else:
						vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,0))
						#vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						rospy.sleep(1.0)
						print("corre do azul berg!")
				else:
					vel = Twist(Vector3(0,0,0), Vector3(0,0,0.5))
					velocidade_saida.publish(vel)
					rospy.sleep(1.0)
					print("TO VENO NADA!")
				#print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
				#print("Centro dos vermelhos: {0}, {1}".format(centro[0], centro[1]))
				#vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
				#print("aqui")
				velocidade_saida.publish(vel)
				rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")


