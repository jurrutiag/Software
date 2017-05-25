#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from geometry_msgs.msg import Point

class Controller():
    def __init__(self):
    
         self.point_sub = rospy.Subscriber('/Point', Point, self.point_callback)
         self.possible_sub = rospy.Subscriber('duckiebot/possible_cmd', Twist2DStamped, self.process_callback)
         self.motor_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
         
         self.motorStopped = Twist2DStamped()
         self.motorStopped.v = 0
         self.motorStopped.omega = 0
         self.Z = 0
         
         
    def process_callback(self,motor):
         
         if self.Z > 10 or self.Z == 0:
              self.motor_pub.publish(motor)
              print('ok')

         else:
              self.motor_pub.publish(self.motorStopped)
              print('not ok')
         
         
    def point_callback(self,point):
         
         self.Z = point.z
         print(self.Z)
         

def main():
    rospy.init_node('controller')
    Controller()
    rospy.spin()

if __name__ == '__main__':
    main()

