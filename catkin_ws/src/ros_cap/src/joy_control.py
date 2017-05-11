#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

def main():
    global base_pub
    global sub
    global msg
    rospy.init_node('test_publisher')
    rospy.loginfo('test_publisher')
    possible_pub = rospy.Publisher('duckiebot/possible_cmd',Twist2DStamped,queue_size=1)
    sub = rospy.Subscriber('/duckiebot/joy', Joy, process_callback)
    msg = Twist2DStamped()
    rospy.spin()

def process_callback(msg1):
    rospy.loginfo(msg1)
    rospy.loginfo(msg1.axes[0])
    msg.header.stamp = rospy.get_rostime()
    lados = msg1.axes[0]
    vertical = msg1.axes[1]
    msg.omega = lados*2
    msg.v = vertical
    possible_pub.publish(msg)

if __name__ == '__main__':
    main()

