#! /usr/bin/env python

import rospy
from amr_detection.srv import Pose_message, Pose_messageRequest
from geometry_msgs.msg import Pose

class EuropalletDetectorClient(object):

    def __init__(self):        
        rospy.wait_for_service("/europallet_detector_server")
        rospy.loginfo("europallet detector service server found")
        self.detection_service = rospy.ServiceProxy("/europallet_detector_server",Pose_message)
        self.request_object = Pose_messageRequest()   
        self.rate = rospy.Rate(5)
        self.delta_pose = Pose()
        rospy.logwarn("constructor done")
        

    def get_delta_pose(self):

        """

        ---
        geometry_msgs/Pose pose       # pose between the base_link and europallet center frame
        string output                 # informations
        bool success                  # it indicates success
        """

        self.result = self.detection_service(self.request_object)
        rospy.loginfo("get_delta_pose")
        
        if self.result.success:
            rospy.loginfo(self.result.success)
            #rospy.logwarn(self.result.output)
            #rospy.loginfo(self.result.pose)
            self.delta_pose = self.result.pose
            self.delta_pose_shifted = self.result.pose_shifted
        else:
            rospy.logwarn(self.result.output)

    
if __name__ == "__main__":    
    rospy.init_node("europallet_service_client_node")
    client_obj = EuropalletDetectorClient()
    client_obj.get_delta_pose()
    rospy.spin()

