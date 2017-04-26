#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

class BaseController(object):
    def __init__(self):
        rospy.wait_for_service('take_image')
        self.sub = rospy.Subscriber('/duckiebot/joy', Joy, self.joy_callback)
        self.pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.cmd = Twist2DStamped()
        self.take_pic = rospy.ServiceProxy('take_image',Empty)

    def joy_callback(self, msg):
        self.cmd.header.stamp = msg.header.stamp
        self.cmd.v = msg.axes[1] * 2
        self.cmd.omega = msg.axes[0] * 0.7
        if msg.buttons[0]:
            resp = self.take_pic()
            print "buena"
        self.pub.publish(self.cmd)

def main():
    rospy.init_node('base_controller')
    rospy.loginfo('Init base controller')
    base = BaseController()   
    rospy.spin()

if __name__ == '__main__':
    main()
