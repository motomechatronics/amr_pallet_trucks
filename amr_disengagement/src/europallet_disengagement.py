#! /usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry


class EuropalletDisengagement(object):

    def __init__(self):


        self.rate = rospy.Rate(5)
        self.y = 0
        self.theta = 0
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist ,queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry , self.callback_odom)

        self.initial_value = False
        self.odometry = Odometry()
        self.cmd_vel = Twist()
        self.phase = "1"  
        self.x0 = 0      
      

    def move_turn(self,velocity,angle,angle_goal):
    
        if angle > angle_goal:
            v = - abs(velocity) * (angle - angle_goal)
        elif angle < angle_goal:
            v = abs(velocity) * (angle_goal - angle)
        else:
            v = 0            
        self.cmd_vel.angular.z = v
        self.cmd_vel_pub.publish(self.cmd_vel)  

    def move_turn1(self,velocity,angle,angle_goal):
        if angle > angle_goal:
            v = - abs(velocity) * (angle - angle_goal)
        elif angle < angle_goal:
            v = abs(velocity) * (angle_goal - angle)
        else:
            v = 0  
        self.cmd_vel.linear.x = 0.05          
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
        if not self.initial_value:
            self.x0 = self.x
            self.initial_value = True

        
        if self.phase == "1":  
            if self.theta > 0:
                goal_angle = 3.14159    
            else:
                goal_angle = - 3.14159   
                
            print("theta",self.theta," goal_angle ", goal_angle)
            print("self.x",self.x," x0 ",self.x0)
            self.move_turn1(0.1,self.theta,goal_angle)
            if self.x < self.x0 - 2:
                self.stop_moving()               
                self.phase = "end"

        #elif self.phase == "2":
           # goal_angle = -1.57
          #  print("theta",self.theta," goal_angle ", goal_angle)
          #  self.move_turn(0.1,self.theta,goal_angle)    
          #  if abs(self.theta - goal_angle) < 0.01:
          #      self.stop_moving() 
          #      rospy.loginfo("disengagement reached")               
           #     self.phase = "end"
        else:
            pass


if __name__ == "__main__":
    rospy.init_node('europallet_disengagement_node')
    rospy.loginfo('europallet disengagement node')
     
    rate = rospy.Rate(10)
    control_c = False

    def shutdownhook():
        global control_c
        rospy.loginfo('shutdown script')
        control_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not control_c:

        EuropalletDisengagement()

        rospy.spin()


      