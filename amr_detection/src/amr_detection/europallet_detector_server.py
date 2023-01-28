#! /usr/bin/env python

import time
import math
import rospy 
from amr_detection.srv import Pose_message, Pose_messageResponse
from europallet_detection import DetectEuropallet
import tf
from sensor_msgs.msg import LaserScan

class EuropalletDetectorServer(object):

    def __init__(self):
        self._ep_detector_service = rospy.Service("/europallet_detector_server", Pose_message, self.service_callback)
        self._response = Pose_messageResponse()
        self._sub = rospy.Subscriber('/rear_scan', LaserScan, self.topic_callback)        
        self.intensity_theshold = 8000       
        self._scan_data = LaserScan()  
        self.ep_frame_published = False
        self.scan_mirror_result = False        
        self._broad_caster_tf = tf.TransformBroadcaster()
        self._listener = tf.TransformListener()
        self._frame_id = "rear_laserscanner_link" #ex: robot_odom
        self._child_frame_id = "base_footprint"    
        self.mean_right_laser_point = 0
        self.mean_left_laser_point = 0 
        time.sleep(1)

    def service_callback(self, request):        
        pose,pose_shifted,success = DetectEuropallet().pub_ep_frame()        
        if success:
            response = Pose_messageResponse()
            """
    
            ---
            geometry_msgs/Pose pose           # pose between the odom link and europallet center frame
            geometry_msgs/Pose pose_shifted   # pose between the odom link and europallet center shifted frame
            string output                     # informations
            bool success                      # it indicates success

            """
            response.pose = pose  
            response.pose_shifted = pose_shifted            
            response.output = "the service call ends successfully"            
            response.success = True
            return response
        else:
            response.pose = pose
            response.pose_shifted = pose_shifted  
            response.output = "the service call is not completed successfully"
            rospy.loginfo(response.output)
            response.success = False
            return response

    def topic_callback(self, msg):
        self.right_lenght_mirror = math.inf
        self.left_lenght_mirror = math.inf
        self._scan_data = msg
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment
        self.samples = math.ceil((self.scan_angle_max - self.scan_angle_min) / self.scan_angle_increment)
        self.scan_angle_full_range_point = self.samples
        self.scan_angle_middle_range_point = self.scan_angle_full_range_point / 2
        self.scantorad = 1/6 * math.pi / 180
        self.scan_angle_middle_range_rad = self.scan_angle_middle_range_point * self.scantorad


if __name__ == "__main__":
    rospy.init_node("europallet_detector_server_node")
    obj = EuropalletDetectorServer()
    rospy.spin()

