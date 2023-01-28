#! /usr/bin/env python

# 1. rotate the amr
# 2. move the amr to the right x position
# 3. rotate the amr
# 4. move the amr forward to the randezvous
# 5. load the europallet
# 6. move the amr backward
# 7. stop to the initial position

import rospy
from geometry_msgs.msg import Twist,Pose,Point
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from amr_detection.srv import Pose_message, Pose_messageRequest
from std_msgs.msg import Float64
import math
import time
import numpy as np

class EuropalletRandezVous(object):

    def __init__(self):

        self.cmd_vel = Twist()
        self.odom = Odometry()  
        self.request_object = Pose_messageRequest()   
        self.rate = rospy.Rate(5)
        self.delta_pose = Pose()
        self.flag = False
        self.x = 0
        self.y = 0
        self.theta = 0
        rospy.wait_for_service("/europallet_detector_server")
        rospy.loginfo("europallet detector service server found")
        self.detection_service = rospy.ServiceProxy("/europallet_detector_server",Pose_message)
        self.get_goal_delta_pose()        
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist ,queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry , self.callback_odom)

        self.initial_value = False
        self.odometry = Odometry()
        self.cmd_vel = Twist()
        self.phase = "1"  
       
  
        self.get_goal_delta_pose()


    def move_forward_y(self,velocity,y,y_goal):      
        if y_goal > y:
            v = abs(velocity) * (y_goal - y)
        elif y > y_goal:
            v = - abs(velocity) * (y - y_goal) 
        else: 
            v = 0
        
        self.cmd_vel.linear.x = v
        self.cmd_vel_pub.publish(self.cmd_vel)      
    
           

    def move_turn(self,velocity,angle,angle_goal):
    
        if angle > angle_goal:
            v = - abs(velocity) * (angle - angle_goal)
        elif angle < angle_goal:
            v = abs(velocity) * (angle_goal - angle)
        else:
            pass
         
        self.cmd_vel.angular.z = v
        self.cmd_vel_pub.publish(self.cmd_vel)  

    def move_turn1(self,velocity,angle,angle_goal):
        if angle > angle_goal:
            v = - abs(velocity) * (angle - angle_goal)
        elif angle < angle_goal:
            v = abs(velocity) * (angle_goal - angle)
        else:
            v = 0  
        self.cmd_vel.linear.x = -0.1          
        self.cmd_vel.angular.z = v
        self.cmd_vel_pub.publish(self.cmd_vel) 

    def stop_moving(self): 
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0       
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)       
        
    def callback_odom(self,msg):     
        
        self.odometry = msg
        self.x = self.odometry.pose.pose.position.x
        self.y = self.odometry.pose.pose.position.y
        q0 = self.odometry.pose.pose.orientation.x
        q1 = self.odometry.pose.pose.orientation.y
        q2 = self.odometry.pose.pose.orientation.z
        q3 = self.odometry.pose.pose.orientation.w
        self.q = [q0,q1,q2,q3] # instantaneous attitude of the amr 
        (roll,pith,yaw) = euler_from_quaternion(self.q)
        self.theta = yaw

        amr_vector = np.array((self.x,self.y)) # vector from origin of odom and origin of base_footprint  odom O ------>O base_footprint
        sep_frame_vector = np.array((self.sxg,self.syg)) # shifted europallet centrer frame vector respect odom
        x_odom = np.array((1,0)) # odom unit x vector
        sep_amr_vector = amr_vector - sep_frame_vector            
        norm_sep_amr_vector = np.linalg.norm(sep_amr_vector)
        inner_prod = sep_amr_vector[0] * x_odom[0]   # sep_amr_vector * x_odom
        cos_sep_amr_vector = inner_prod / norm_sep_amr_vector 
        angle_sep_amr_vector = math.acos(cos_sep_amr_vector)         
        goal_angle = angle_sep_amr_vector
        
        if self.phase == "1":  
            
            goal_angle = abs(self.thetag) - 1.57 #3.14159            
            #goal_angle = self.thetag # 1.57        
            print("theta",self.theta," goal_angle ",goal_angle)
            self.move_turn(0.1,self.theta,goal_angle)
            if abs(self.theta - goal_angle) < 0.03:
                self.stop_moving()
                time.sleep(2)
                self.phase = "2"

        elif self.phase == "2":
            print("self.y",self.y," self.syg ",self.syg)  
                     
            self.move_forward_y(0.5,self.y,self.syg)
            if abs(self.y - self.syg) < 0.01:
                self.stop_moving()
                time.sleep(2)
                self.phase = "3"
            
        elif self.phase == "3":
            if self.theta > 0:
                goal_angle = 3.14159 #abs(self.thetag)  #3.14159
            else:
                goal_angle = -3.14159 #-abs(self.thetag)  #-3.14159
            # goal_angle = self.thetag
            rospy.loginfo(self.phase)
            print("theta",self.theta," goal_angle ",goal_angle)
            self.move_turn(0.3,self.theta,goal_angle) 
            if abs(self.theta - goal_angle) < 0.01:
                self.move_turn1(0.3,self.theta,goal_angle)
                #i = -abs(self.xg - self.x) / abs(self.xg)    
                #self.cmd_vel.linear.x = -0.25 #* math.exp(i) - 0.02
                #self.cmd_vel_pub.publish(self.cmd_vel) 

                if self.x > self.xg - 0.3: 
                    self.stop_moving()
                    self.phase = "5"


        elif self.phase == "5":
            if not self.initial_value:
                rospy.loginfo(self.phase)
                rospy.loginfo("approach reached")
                self.initial_value = True
                 
        

             
    
       
       
    def get_goal_delta_pose(self):   

        self.result = self.detection_service(self.request_object)
        #rospy.loginfo(self.result.pose)
             
        if self.result.success:
            goal_delta_pose = Pose()

            #goal_delta_pose.position.x # distance of the base_footprint and the ep_cf frames a long the direction ortogonal to the 
                                            # longitudinal direction of the europallet


            #goal_delta_pose.position.y # distance of the base_footprint and the ep_cf frames a long the   
                                            # longitudinal direction of the europallet
           
            goal_delta_pose = self.result.pose            

            self.xg = goal_delta_pose.position.x
            self.yg = goal_delta_pose.position.y                                      
            q0 = goal_delta_pose.orientation.x
            q1 = goal_delta_pose.orientation.y
            q2 = goal_delta_pose.orientation.z
            q3 = goal_delta_pose.orientation.w
            q_goal = [q0,q1,q2,q3]
            (roll,pith,yaw) = euler_from_quaternion(q_goal)
            self.thetag = yaw

            goal_delta_pose_shifted = self.result.pose_shifted
            self.sxg = goal_delta_pose_shifted.position.x
            self.syg = goal_delta_pose_shifted.position.y                                      
            sq0 = goal_delta_pose_shifted.orientation.x
            sq1 = goal_delta_pose_shifted.orientation.y
            sq2 = goal_delta_pose_shifted.orientation.z
            sq3 = goal_delta_pose_shifted.orientation.w
            sq_goal = [sq0,sq1,sq2,sq3]
            (sroll,spith,syaw) = euler_from_quaternion(sq_goal)
            self.sthetag = syaw
            
        else:
            rospy.logwarn(self.result.output)



if __name__ == "__main__":
    rospy.init_node('europallet_randezvous_node')
    rospy.loginfo('europallet randezvous node')
     
    rate = rospy.Rate(10)
    control_c = False

    def shutdownhook():
        global control_c
        rospy.loginfo('shutdown script')
        control_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not control_c:

        EuropalletRandezVous()

        rospy.spin()


      