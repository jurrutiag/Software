#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from geometry_msgs.msg import Point
def main():
    global base_pub
    global sub
    global msg
    rospy.init_node('test_publisher')
    rospy.loginfo('test_publisher')
    base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    base_sub = rospy.Subscriber('/Point', Point, process_callback)
    msg = Twist2DStamped()
    rospy.spin()

def process_callback(msg1):
    rospy.loginfo(msg1)
    rospy.loginfo(msg1.x)
    msg.header.stamp = rospy.get_rostime()
    X = msg1.x
    if(X>325):
         msg.omega = 1
    elif(X<320 and X>0):
         msg.omega = -1
    else:
         msg.omega = 0 
    print msg.omega
    base_pub.publish(msg)

if __name__ == '__main__':
    main()


