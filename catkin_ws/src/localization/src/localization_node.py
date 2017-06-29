#!/usr/bin/env python
import rospy
#from apriltags_ros.msg import AprilTagDetectionArray
from duckietown_msgs.msg import AprilTagsWithInfos
import tf2_ros
from tf2_msgs.msg import TFMessage
import tf.transformations as tr
from geometry_msgs.msg import Transform, TransformStamped
import numpy as np
from localization import PoseAverage
from visualization_msgs.msg import Marker
import time
import math
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

# Localization Node
# Author: Teddy Ort
# Inputs: apriltags/duckietown_msgs/AprilTags - A list of april tags in a camera frame
# Outputs: pose2d/duckietown_msgs/Pose2dStamped - The estimated pose of the robot in the world frame in 2D coordinates
#          pose3d/geometry_msgs/PoseStamped - The estimated pose of the robot in the world frame in 3D coordinates

class LocalizationNode(object):
    def __init__(self):
        self.node_name = 'localization_node'

        # Constants
        self.world_frame = "world"
        self.duckiebot_frame = "duckiebot"
        self.time0 = time.time()
        self.time1 = time.time()
        self.timeDif = 0
        self.motorAnterior = Twist2DStamped()
        self.counter = 0
        self.omega = 0
        self.x = 0
        self.y = 0
        
        #self.transActual = Transform()
        
        self.komega = 5.19/13
        self.kvel = 0.62/2
        self.last_world_frame = "world"
        self.last_duckiebot_frame = "duckiebot"

        self.duckiebot_lifetime = self.setupParam("~duckiebot_lifetime", 5) # The number of seconds to keep the duckiebot alive bewtween detections
        self.highlight_lifetime = self.setupParam("~highlight_lifetime", 3) # The number of seconds to keep a sign highlighted after a detection

        # Setup the publishers and subscribers
        self.sub_april = rospy.Subscriber("~apriltags", AprilTagsWithInfos, self.tag_callback)
        self.sub_motor = rospy.Subscriber('/duckiebot/wheels_driver_node/car_cmd',Twist2DStamped,self.motor_callback)
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1, latch=True)
        self.pub_rviz = rospy.Publisher("/sign_highlights", Marker, queue_size=1, latch=True)

        # Setup the transform listener
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        # Use a timer to make the duckiebot disappear
        self.lifetimer = rospy.Time.now()
        #self.publish_duckie_marker()

        

        rospy.loginfo("[%s] has started", self.node_name)

    def tag_callback(self, msg_tag):
        # Listen for the transform of the tag in the world
        avg = PoseAverage.PoseAverage()
        for tag in msg_tag.detections:
            try:
                Tt_w = self.tfbuf.lookup_transform(self.world_frame, "tag_{id}".format(id=tag.id), rospy.Time(), rospy.Duration(1))
                Mtbase_w=self.transform_to_matrix(Tt_w.transform)
                Mt_tbase = tr.concatenate_matrices(tr.translation_matrix((0,0,0.17)), tr.euler_matrix(0,0,np.pi))
                Mt_w = tr.concatenate_matrices(Mtbase_w,Mt_tbase)
                Mt_r=self.pose_to_matrix(tag.pose)
                Mr_t=np.linalg.inv(Mt_r)
                Mr_w=np.dot(Mt_w,Mr_t)
                Tr_w = self.matrix_to_transform(Mr_w)
                avg.add_pose(Tr_w)
                self.publish_sign_highlight(tag.id)
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                rospy.logwarn("Error looking up transform for tag_%s", tag.id)
                rospy.logwarn(ex.message)

        Tr_w =  avg.get_average() # Average of the opinions

        # Broadcast the robot transform
        if Tr_w is not None:
            # Set the z translation, and x and y rotations to 0
            Tr_w.translation.z = 0
            rot = Tr_w.rotation
            rotz=tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))[2]
            (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_euler(0, 0, rotz)
            T = TransformStamped()
            T.transform = Tr_w
            #self.transActual = Tr_w
            T.header.frame_id = self.world_frame
            self.last_world_frame = self.world_frame
            T.header.stamp = rospy.Time.now()
            T.child_frame_id = self.duckiebot_frame
            self.last_duckiebot_frame = self.duckiebot_frame
            self.x = Tr_w.translation.x
            self.y = Tr_w.translation.y
            self.omega = rotz
            self.pub_tf.publish(TFMessage([T]))
            self.lifetimer = rospy.Time.now()
            

    def publish_duckie_marker(self):
        # Publish a duckiebot transform far away unless the timer was reset
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            if rospy.Time.now() - self.lifetimer > rospy.Duration(self.duckiebot_lifetime):
                T = TransformStamped()
                T.transform.translation.z = 1000    # Throw it 1km in the air
                T.transform.rotation.w = 1
                T.header.frame_id = self.world_frame
                T.header.stamp = rospy.Time.now()
                T.child_frame_id = self.duckiebot_frame
                self.pub_tf.publish(TFMessage([T]))

    def publish_sign_highlight(self, id):
        # Publish a highlight marker on the sign that is seen by the robot
        m = Marker()
        m.header.frame_id="tag_{id}".format(id=id)
        m.header.stamp = rospy.Time.now()
        m.id=id
        m.lifetime = rospy.Duration(self.highlight_lifetime)
        m.type = Marker.CYLINDER
        p = m.pose.position
        o = m.pose.orientation
        c = m.color
        s = m.scale
        s.x, s.y, s.z = (0.1, 0.1, 0.3)
        p.z = 0.15
        c.a, c.r, c.g, c.b = (0.2, 0.9, 0.9, 0.0)
        o.w = 1
        self.pub_rviz.publish(m)

    def pose_to_matrix(self, p):
        # Return the 4x4 homogeneous matrix for a PoseStamped.msg p from the geometry_msgs
        trans = (p.pose.position.x, p.pose.position.y, p.pose.position.z)
        rot = (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
        return np.dot(tr.translation_matrix(trans), tr.quaternion_matrix(rot))

    def transform_to_matrix(self, T):
        # Return the 4x4 homogeneous matrix for a TransformStamped.msg T from the geometry_msgs
        trans = (T.translation.x, T.translation.y, T.translation.z)
        rot = (T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w)
        return np.dot(tr.translation_matrix(trans), tr.quaternion_matrix(rot))

    def matrix_to_transform(self, M):
        # Return a TransformStamped.msg T from the geometry_msgs from a 4x4 homogeneous matrix
        T=Transform()
        (T.translation.x, T.translation.y, T.translation.z) = tr.translation_from_matrix(M)
        (T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w) = tr.quaternion_from_matrix(M)
        return T

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value
    
    def motor_callback(self,motor):
        
        if self.counter == 0:
            self.time0 = rospy.get_time()
            self.counter = 1
            self.motorAnterior = motor
        elif self.counter == 1:
            self.time1 = rospy.get_time()
            self.timeDif = self.time1-self.time0
            self.counter = 0
            deltadist = self.motorAnterior.v*self.kvel*self.timeDif
            deltaomega = self.motorAnterior.omega*self.komega*self.timeDif
            
            self.omega = self.omega+deltaomega
            self.omega = self.reiniciarAngulo(self.omega)
            self.x = self.x + deltadist*math.cos(self.omega)
            self.y = self.y + deltadist*math.sin(self.omega)
            rospy.loginfo(self.omega)
            rospy.loginfo(self.timeDif)
            #MatrixInicial = self.transform_to_matrix(self.transActual)
            #DeltaMatrix = np.matrix([[math.cos(self.omega),-math.sin(self.omega),0,deltadist],[math.sin(self.omega),math.cos(self.omega),0,0],[0,0,1,0],[0,0,0,1]])
            #MatrixFinal = np.dot(MatrixInicial,DeltaMatrix)
            #TransformFinal = self.matrix_to_transform(MatrixFinal)
            
            transformAprox = Transform()
            #transformAprox = MatrixFinal
            transformAprox.translation.x = self.x
            transformAprox.translation.y = self.y
            transformAprox.translation.z = 0
            (transformAprox.rotation.x,transformAprox.rotation.y,transformAprox.rotation.z,transformAprox.rotation.w) = tr.quaternion_from_euler(0,0,self.omega)
            T2 = TransformStamped()
            T2.transform = transformAprox
            T2.header.frame_id = self.last_world_frame
            T2.header.stamp = rospy.Time.now()
            T2.child_frame_id = self.last_duckiebot_frame
            
            rospy.loginfo(TFMessage([T2]))
            self.pub_tf.publish(TFMessage([T2]))
            

    def reiniciarAngulo(self,angulo):
        if abs(angulo)>=2*math.pi:
            if angulo>=0:
                angulo-=2*math.pi
            elif angulo < 0:
                angulo+=2*math.pi
        return angulo


if __name__ == '__main__':
    rospy.init_node('localization_node', anonymous=False)
    localization_node = LocalizationNode()
    rospy.spin()
