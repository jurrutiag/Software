#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# define range of blue color in HSV
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([0,50,50])
upper_red = np.array([25,255,255])
lower_yellow = np.array([25,150,100])
upper_yellow = np.array([35,255,255])



class BlobColor():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self._process_image)
        self.image_publisher = rospy.Publisher('/hola', Image, queue_size=1)
        self.point_publisher = rospy.Publisher('/Point', Point, queue_size=1)
        self.mask_publisher = rospy.Publisher('/mask', Image, queue_size=1)
        self.motor_publisher = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.msg = Twist2DStamped()

        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 10



    def _process_image(self,img):
        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        #Se deja en frame la imagen actual
        frame = self.cv_image

        #Cambiar tipo de color de BGR a HSV
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        # segment_image = cv2.bitwise_and(frame,frame, mask= mask)
        segment_image = cv2.bitwise_and(frame,frame, mask= mask)
        final_image = self.bridge.cv2_to_imgmsg(segment_image,"bgr8")
        


        kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
        img_out = cv2.erode(mask, kernel, iterations = 3)
        
        #Operacion morfologica dilate
        img_out_final = cv2.dilate(img_out, kernel, iterations = 1)

        image, contours, hierarchy = cv2.findContours(img_out_final,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        #vectorX = []
        #vectorY = []
        #vectorWH = []
        #vectorW = []
        #vectorH = []
        wh = 0
        xPoint = 0
        yPoint = 0
        hPoint = 0
        wPoint = 0
        for cnt in contours:
            #Obtener rectangulo
            x,y,w,h = cv2.boundingRect(cnt)

            #Filtrar por area minima
            if w*h > self.min_area:

                #Dibujar un rectangulo en la imagen
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,0), 2)              
                if w*h>=wh:
                     wh = w*h
                     xPoint = x
                     yPoint = y
                     wPoint = w
                     hPoint = h
                #vectorX.append(x)
                #vectorY.append(y)
                #vectorWH.append(w*h)
                #vectorW.append(w)
                #vectorH.append(h)             
            
        #iwh = vectorWH.index(max(vectorWH))
        #xPoint = vectorX[iwh]
        #yPoint = vectorY[iwh]
        #wPoint = vectorW[iwh]
        #hPoint = vectorH[iwh]
        P = Point()
        P.x = xPoint + wPoint/2
        P.y = yPoint + hPoint/2
        #Publicar frame
#640X480bgr
        mask2 = cv2.cvtColor(img_out_final,cv2.COLOR_GRAY2BGR)
        frame2 = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        imagenPrueba = self.bridge.cv2_to_imgmsg(mask2, "bgr8")

        if(P.x>325):
             self.msg.omega = 1
        elif(P.x<320 and P.x>0):
             self.msg.omega = -1
        else:
             self.msg.omega = 0 
        print self.msg.omega
        self.motor_publisher.publish(self.msg)
        #self.mask_publisher.publish(imagenPrueba)
        self.image_publisher.publish(frame2)
        self.point_publisher.publish(P)

        #Publicar Point center de mayor tamanio

def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()
