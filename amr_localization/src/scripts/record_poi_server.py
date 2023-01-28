#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from amr_localization.srv import Poi_message, Poi_messageResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml


class SavePoi(object):
    def __init__(self):
        self.save_poi_service = rospy.Service(
            "/save_poi", Poi_message, self.service_callback)
        rospy.loginfo("launching server...") 
        rospy.logwarn("use a terminal to save the position with: rosservice call /save_poi TAB-TAB")
        self.sub = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.subscriber_callback)
        self.response = Poi_messageResponse()
        self.dict = {}

    def subscriber_callback(self, msg):

        # position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        # orientation
        self.a = msg.pose.pose.orientation.x
        self.b = msg.pose.pose.orientation.y
        self.g = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w

    def save_pose(self):
        rospy.loginfo("data saved.")
        with open('/home/user/catkin_ws/src/amr_localization/src/scripts/poi.yaml', 'w') as file:
            yaml.dump(self.dict, file)

    def add_pose(self):
        self.item = {self.data: {
                     'position': {
                         'x': self.x,
                         'y': self.y,
                         'z': self.z},
                     'orientation': {
                         'x': self.a,
                         'y': self.b,
                         'z': self.g,
                         'w': self.w}}}

        self.dict = {**self.item, **self.dict}
        print(self.dict)

    def service_callback(self, request):
        self.data = request.label

        if not self.data == "end":
            self.add_pose()
            self.response.output = ""
            return self.response

        else:
            self.save_pose()
            self.response.output = "pose/s correctly saved"
            return self.response


if __name__ == "__main__":
    rospy.init_node("save_poi_service_server_node")
    try:
        SavePoi()
        rospy.spin()
    except rospy.ROSInitException:
        pass
