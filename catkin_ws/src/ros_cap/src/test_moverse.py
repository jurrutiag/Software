#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from geometry_msgs.msg import Point

class test_giro():
    
    def __init__(self):
        
        self.motor_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.point_sub = base_sub = rospy.Subscriber('/Point', Point, self.process_callback)
        self.motorMsg = Twist2DStamped()
        
    def process_callback(point):
        X = point.x
    	if(X>325):
            self.motorMsg.omega = 1
        elif(X<320 and X>0):   
            self.motorMsg.omega = -1
        else:
            self.motorMsg.omega = 0 
        
        base_pub.publish(self.motorMsg)
    
def main():
    rospy.init_node('test_giro')
    test_giro()
    rospy.spin()

if __name__ == '__main__':
    main()


