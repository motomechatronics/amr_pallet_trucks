#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class InitializePose(object):
    def __init__(self):
        self._posedata = PoseWithCovarianceStamped()
        self._odomdata = Odometry()
        self._pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(1)
        self.flag = True

    def odom_callback(self, msg):
        self._odomdata = msg
        # rospy.loginfo(self._odomdata)
    
    def get_odometry(self):        
        return self._odomdata
        
        """
        rosmsg info nav_msgs/Odometry
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string child_frame_id
        geometry_msgs/PoseWithCovariance pose
        geometry_msgs/Pose pose
            geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
            geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        float64[36] covariance
        geometry_msgs/TwistWithCovariance twist
        geometry_msgs/Twist twist
            geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
            geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
        float64[36] covariance

        """
    def set_pose(self): 

        while not rospy.is_shutdown():
            connections =  self._pose_pub.get_num_connections()            
            if connections > 0 and self.flag:
                _odom = self.get_odometry()            
                self._posedata.header.frame_id = "map"
                self._posedata.pose.pose.position.x = _odom.pose.pose.position.x
                self._posedata.pose.pose.position.y =  _odom.pose.pose.position.y
                self._posedata.pose.pose.position.z =  _odom.pose.pose.position.z
                self._posedata.pose.pose.orientation.x = _odom.pose.pose.orientation.x
                self._posedata.pose.pose.orientation.y = _odom.pose.pose.orientation.y
                self._posedata.pose.pose.orientation.z = _odom.pose.pose.orientation.z
                self._posedata.pose.pose.orientation.w = _odom.pose.pose.orientation.w
                self._pose_pub.publish(self._posedata)
                rospy.logwarn("n. %d connection to /initialopose topic...publishing", connections)
                
                self.flag =  False
            else:
                pass
            self.rate.sleep()   
  
if __name__ == "__main__":
    rospy.init_node("amr_initialpose_node")
    try:
        obj = InitializePose()
        obj.set_pose()
        rospy.spin()

    except rospy.ROSInitException:
        pass