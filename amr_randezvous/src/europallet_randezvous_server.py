#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from europallet_randezvous import EuropalletRandezVous
#from geometry_msgs.msg import PoseStamped
#import yaml


class EuroPalletRandezvousService(object):

    def __init__(self):
        self._randezvous_service = rospy.Service(
            "/randezvous", Trigger, self.service_callback)
        self._response = TriggerResponse()
        """
        ---
        bool success   # indicate successful run of triggered service
        string message # informational, e.g. for error messages
        """

    def service_callback(self,request):       
        
        self._response.success = EuropalletRandezVous()
        if self._response.success:            
            self._response.message = "OK"
            return self._response

    


if __name__ == "__main__":

    rospy.init_node("randezvous_service_node")
    try:
        obj = EuroPalletRandezvousService()
        rospy.spin()

    except rospy.ROSInitException:
        pass
