#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from geometry_msgs.msg import Point


def main():
    rospy.init_node('controller')
    global motorStopped
    global Z
    Z = 10
    global wheels_pub
    wheels_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    Point_sub = rospy.Subscriber('/Point', Point, point_callback)
    Possible = rospy.Subscriber('duckiebot/possible_cmd', Twist2DStamped, process_callback)
    motorStopped = Twist2DStamped()
    motorStopped.v = 0
    motorStopped.omega = 0
    rospy.spin()

def process_callback(msg1):
    if Z>10 or Z==0:
        wheels_pub.publish(msg1)
        print Z
        
    else:
        wheels_pub.publish(motorStopped)
        print(motorStopped)

def point_callback(point):
    Z = point.z
    
    

if __name__ == '__main__':
    main()
