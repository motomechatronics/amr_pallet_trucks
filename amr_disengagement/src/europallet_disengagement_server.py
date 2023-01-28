#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from europallet_disengagement import EuropalletDisengagement
#from geometry_msgs.msg import PoseStamped
#import yaml


class EuroPalletDisengagementService(object):

    def __init__(self):
        self._disengagement_service = rospy.Service(
            "/disengagement", Trigger, self.service_callback)
        self._response = TriggerResponse()
        """
        ---
        bool success   # indicate successful run of triggered service
        string message # informational, e.g. for error messages
        """

    def service_callback(self,request):       
        
        self._response.success = EuropalletDisengagement()
        if self._response.success:            
            self._response.message = "OK"
            return self._response

    


if __name__ == "__main__":

    rospy.init_node("disengagement_service_node")
    try:
        obj = EuroPalletDisengagementService()
        rospy.spin()

    except rospy.ROSInitException:
        pass
