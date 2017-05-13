#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

def main():
    wheels_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    Point_sub = rospy.Subscriber('/Point', Point, point_callback)
    Possible = rospy.Subscriber('duckiebot/possible_cmd', Twist2DStamped, process_callback)
    motor = Twist2DStamped()
    rospy.spin()

def process_callback(msg1):
    Possible.publish(msg1)
    

if __name__ == '__main__':
    main()
