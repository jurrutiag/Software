#!/usr/bin/env python
from sensor_msgs.msg import Image

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

class joy_control():
    
    def __init__(self):
        
        #self.possible_pub = rospy.Publisher('duckiebot/possible_cmd',Twist2DStamped,queue_size=1)
        self.normal_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd',Twist2DStamped,queue_size=1)
        self.joy_sub = rospy.Subscriber('/duckiebot/joy', Joy, self.process_callback)
        self.msgMotor = Twist2DStamped()
        
        #prueba sin joystick, usando camara
        #self.subExtra = rospy.Subscriber('/usb_cam/image_raw', Image, self.process_callback2)
        
    def process_callback(self,msg):
        lr = msg.axes[0]
        bwd = msg.axes[2] #presionando el boton de presion izquierdo
        fwd = msg.axes[5] #presionando el boton de presion derecho
        self.msgMotor.v = (abs(fwd-1) + (bwd-1))/2
        self.msgMotor.omega = lr*13
        if abs(self.msgMotor.omega) <= 7.5:
            self.msgMotor.omega = 0
        print(self.msgMotor)
        #rospy.loginfo(self.msgMotor.v)
        #rospy.loginfo(self.msgMotor.omega)
        #self.possible_pub.publish(self.msgMotor)
        self.normal_pub.publish(self.msgMotor)
    #eliminar luego de tener el robot    
#    def process_callback2(self,Imagen):
#        self.msgMotor.v = 5
#        self.possible_pub.publish(self.msgMotor)



def main():
    rospy.init_node('joy_control')
    joy_control()
    rospy.spin()

if __name__ == '__main__':
    main()

