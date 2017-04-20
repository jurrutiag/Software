#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError


class TakeImage():

    def __init__(self):

        self.namespace_prefix = rospy.get_namespace()
        self.image_service = rospy.Service(self.namespace_prefix+'take_image', Empty, self._take_image)

        #subscribe image
        self.image_subscriber = None 

        self.bridge = CvBridge()
        self.cv_image = Image()
        
        self.cont = 0

        self.image_subscriber = rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self._process_image)


    def _process_image(self,img):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        # cv2.imshow("Image window", self.cv_image)
        # cv2.waitKey(3)



    def _take_image(self, req):
        print "master"
        rgbimage = self.cv_image
        cv2.imwrite("image_"+str(self.cont)+".png",rgbimage)

        self.cont += 1
        print "buena"
        return EmptyResponse()

def main():

    rospy.init_node('TakeImage')

    TakeImage()

    rospy.spin()

if __name__ == '__main__':
    main()
