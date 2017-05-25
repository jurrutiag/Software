#!/usr/bin/env python
from sensor_msgs.msg import Image

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

def main():
    global base_pub
    global sub
    global msg
    global possible_pub
    rospy.init_node('test_publisher')
    rospy.loginfo('test_publisher')
    possible_pub = rospy.Publisher('duckiebot/possible_cmd',Twist2DStamped,queue_size=1)
    sub = rospy.Subscriber('/duckiebot/joy', Joy, process_callback)
    global msgMotor
    msgMotor = Twist2DStamped()

    subExtra = rospy.Subscriber('/usb_cam/image_raw', Image, process_callback2)
    rospy.spin()
    

def process_callback(msg1):
    rospy.loginfo(msg1)
    rospy.loginfo(msg1.axes[0])
    msgMotor.header.stamp = rospy.get_rostime()
    lados = msg1.axes[0]
    vertical = msg1.axes[1]
    msgMotor.omega = lados*2
    msgMotor.v = vertical
    possible_pub.publish(msgMotor)

def process_callback2(Imagen):
    msgMotor.v = 5
    possible_pub.publish(msgMotor)

if __name__ == '__main__':
    main()

