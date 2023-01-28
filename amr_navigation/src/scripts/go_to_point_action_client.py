#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from geometry_msgs.msg import PoseStamped


class GoToPointActionClient(object):

    def __init__(self, go_to_point):

        self.go_to_point_client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)
        rospy.loginfo("wainting for move_base action server")
        self.go_to_point_client.wait_for_server()
        rospy.loginfo("move_base action server found")
        # creates a goal to send to the action server
        self.goal = MoveBaseGoal()
        self.goal.target_pose = go_to_point
        # sends the goal to the action server, specifying which feedback function
        # to call when feedback received
        self.go_to_point_client.send_goal(
            self.goal, feedback_cb=self.feedback_callback)

    def result_reached(self):
        self.result = self.go_to_point_client.wait_for_result()
        return self.result

    def feedback_callback(self, feedback):
        rospy.loginfo("goal received from move_base server")


if __name__ == '__main__':
    rospy.init_node("go_to_point_action_client_node")
    from geometry_msgs.msg import PoseStamped
    try:
        go_to_point = PoseStamped()
        go_to_point.header.frame_id = "map"
        go_to_point.pose.position.x = 0
        go_to_point.pose.position.y = 0
        go_to_point.pose.position.z = 0
        go_to_point.pose.orientation.x = 0
        go_to_point.pose.orientation.y = 0
        go_to_point.pose.orientation.z = 0
        go_to_point.pose.orientation.w = 1

        GoToPointActionClient(go_to_point)
        if (GoToPointActionClient(go_to_point).result_reached()):
            print('goal reached')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
