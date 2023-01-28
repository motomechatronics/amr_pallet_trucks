#! /usr/bin/env python

import rospy
from amr_localization.srv import Poi_message, Poi_messageResponse
from go_to_point_action_client import GoToPointActionClient
from geometry_msgs.msg import PoseStamped
import yaml


class GoToPoint(object):

    def __init__(self):
        self._go_to_points_service = rospy.Service(
            "/go_to_point", Poi_message, self.service_callback)
        self._goal = PoseStamped()
        self._go_to_point_ac_object = GoToPointActionClient(self._goal)
        #self._label = ""
        #self._success = False
        self._response = Poi_messageResponse()

    def service_callback(self, request):

        # question: how can I assemble the service and the action client
        # importing them in a third file as main_navigation.py file?
        self._label = request.label
        # rospy.loginfo(self._label)
        pose = self.load_pose()
        GoToPointActionClient(pose)
        success = GoToPointActionClient(
            pose).go_to_point_client.wait_for_result()
        if success:
            response = Poi_messageResponse()
            """
            string label # may be the label of the position or the "end" command to save the pose
            ---
            string output # indicate the file poi.yaml saved
            """
            response.output = "OK"
            return response

    def load_pose(self):

        # question: how can I write the load_pose
        # function in a more compact way?
        self._goal.header.frame_id = "map"
        param_name = f"/{self._label}/position/x"
        # rospy.loginfo(param_name)
        self._goal.pose.position.x = rospy.get_param(param_name)

        param_name = f"/{self._label}/position/y"
        self._goal.pose.position.y = rospy.get_param(param_name)

        param_name = f"/{self._label}/position/z"
        self._goal.pose.position.z = rospy.get_param(param_name)

        param_name = f"/{self._label}/orientation/x"
        self._goal.pose.orientation.x = rospy.get_param(param_name)

        param_name = f"/{self._label}/orientation/y"
        self._goal.pose.orientation.y = rospy.get_param(param_name)

        param_name = f"/{self._label}/orientation/z"
        self._goal.pose.orientation.z = rospy.get_param(param_name)

        param_name = f"/{self._label}/orientation/w"
        self._goal.pose.orientation.w = rospy.get_param(param_name)

        self.success = True
        rospy.loginfo(self._goal)
        return self._goal


if __name__ == "__main__":
    rospy.init_node("main_navigation_node")
    try:

        obj = GoToPoint()
        rospy.spin()

    except rospy.ROSInitException:
        pass
