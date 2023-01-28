#! /usr/bin/env python

import rospy
from amr_description.srv import ElevatorServiceMessage, ElevatorServiceMessageResponse
from std_msgs.msg import Float64

class ElevatorServiceServer():

    def __init__(self):
        rospy.loginfo("elevator service server running...")
        self.elevator_service_server = rospy.Service(
            '/elevator_service_server', ElevatorServiceMessage, self.service_callback)

        self.fork_sx_pub = rospy.Publisher(
            '/amr/base_link_elevator_dx_link_joint_position_controller/command', Float64, queue_size=1)

        self.fork_dx_pub = rospy.Publisher(
            '/amr/base_link_elevator_sx_link_joint_position_controller/command', Float64, queue_size=1)

        self.setpoint = Float64()
        self.rate = rospy.Rate(10)  # 10hz
        self.ctrl_c = False
        self.elevator_service_response = ElevatorServiceMessageResponse()       
     

    def service_callback(self, request):

        self.elevator_service_response.success = False
        self.printed_flag = True

        while not self.elevator_service_response.success:

            if request.elevator == "up":
                self.setpoint.data = 0.120
                connections_sx =  self.fork_sx_pub.get_num_connections()
                connections_dx =  self.fork_dx_pub.get_num_connections()
                print(connections_dx,connections_sx)
                if connections_sx > 0 and connections_dx > 0:
                    self.fork_sx_pub.publish(self.setpoint)
                    self.fork_dx_pub.publish(self.setpoint) 
                    self.elevator_service_response.success = True  
                    rospy.loginfo("the elevator stutus is up.")
                else:
                     rospy.logwarn("No connactions: verify that the service server is running.")             

            elif request.elevator == "down":
                self.setpoint.data = 0.0
                connections_sx =  self.fork_sx_pub.get_num_connections()
                connections_dx =  self.fork_dx_pub.get_num_connections()
                if connections_sx > 0 and connections_dx > 0:
                    self.fork_sx_pub.publish(self.setpoint)
                    self.fork_dx_pub.publish(self.setpoint) 
                    self.elevator_service_response.success = True  
                    rospy.loginfo("the elevator stutus is down.")    
                else:
                     rospy.logwarn("No connactions: verify that the service server is running.")        
            
            else: 
                if self.printed_flag:
                    rospy.logwarn('Not proper keywords. Use keywork "up" or "down" to move the forks.')
                    self.printed_flag = False
                    break
        
        return self.elevator_service_response


if __name__ == '__main__':
    rospy.init_node('elevator_service_server_node', anonymous=True)

    try:
        server_object = ElevatorServiceServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass