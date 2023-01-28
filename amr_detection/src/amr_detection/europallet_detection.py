#! /usr/bin/env python

import math
import time
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class DetectEuropallet(object):
    def __init__(self):        
        self._sub = rospy.Subscriber('/rear_scan', LaserScan, self.topic_callback)        
        self.intensity_theshold = 8000       
        self._scan_data = LaserScan()     
        self.cart_frame_published = False
        self.scan_mirror_result = False        
        self._broad_caster_tf = tf.TransformBroadcaster()
        self._listener = tf.TransformListener()
        self._frame_id = "rear_laserscanner_link" #ex: robot_odom
        self._child_frame_id = "base_footprint"    
        self.mean_right_laser_point = 0
        self.mean_left_laser_point = 0      
        time.sleep(1)        

    def topic_callback(self, msg):
        self.right_lenght_mirror = math.inf
        self.left_lenght_mirror = math.inf
        self._scan_data = msg
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment
        self.samples = math.ceil((self.scan_angle_max - self.scan_angle_min) / self.scan_angle_increment)
        self.scan_angle_full_range_point = self.samples
        self.scan_angle_middle_range_point = self.scan_angle_full_range_point / 2
        self.scantorad = 1/6 * math.pi / 180
        self.scan_angle_middle_range_rad = self.scan_angle_middle_range_point * self.scantorad

    def scan_mirrors(self):        
        # procede if _scan_data.intensities is not empty
        if len(self._scan_data.intensities) > 0:
            #self.scan_mirrors_index_list = []  
            self.scan_mirrors_point_list = []          
            for i in range(0, self.samples):                
                if self._scan_data.intensities[i] == self.intensity_theshold:
                    self.scan_mirrors_point = [i]     
                    # the following list contains all the reflectives points               
                    self.scan_mirrors_point_list = self.scan_mirrors_point_list + self.scan_mirrors_point
                    
            # laser
                                                 ##########
                                               ##############
                                              ################
                                            ###################
            # left point 720 (max. angle)  #####################  right point 0 (min. angle)

            if len(self.scan_mirrors_point_list) > 0:  # if list is not empty
                max_laser_reflective_point = max(self.scan_mirrors_point_list)
                min_laser_reflective_point = min(self.scan_mirrors_point_list)
                # find the mean point between the min and max points of the list
                mean_laser_reflective_point = int((min_laser_reflective_point + max_laser_reflective_point) / 2)
                right_laser_reflective_list = []
                left_laser_reflective_list = []
                # divide the points in two groups: left and right                 
                for i, value in enumerate(self.scan_mirrors_point_list):
                    if value <=  mean_laser_reflective_point:
                        right_laser_reflective_list.append(self.scan_mirrors_point_list[i])
                    else:
                        left_laser_reflective_list.append(self.scan_mirrors_point_list[i])
                # for each group it finds the mean point
                if len(right_laser_reflective_list) > 0:                
                    self.mean_right_laser_point = int(sum(right_laser_reflective_list)/len(right_laser_reflective_list))
                else: 
                    self.mean_right_laser_point = 0

                if len(left_laser_reflective_list) > 0:
                    self.mean_left_laser_point = int(sum(left_laser_reflective_list)/len(left_laser_reflective_list))
                else:
                    self.mean_left_laser_point = 0

                # rospy.loginfo([right_laser_reflective_list,left_laser_reflective_list])
            
            # sample the distance from the rear_laserscaner_lin frame and the right and left mirrors
            self.scan_mirrors_left_beam = self._scan_data.ranges[self.mean_left_laser_point]  
            self.scan_mirrors_right_beam = self._scan_data.ranges[self.mean_right_laser_point]
            # sample the angle between the right and left mirrors distances
          

            # saturates the distance at 20m
            if self.scan_mirrors_left_beam == math.inf:
                self.scan_mirrors_left_beam = 20
            if self.scan_mirrors_right_beam == math.inf:
                self.scan_mirrors_right_beam = 20

            # transform index to angle [rad] 
            # the following formulas are refered to the triangole OAB between respectively rear_laserscanner_frames (O) , left mirror freme origin (A) and right mirror frame origin (B)
            self.scan_mirrors_angle_left_beam = self.scan_angle_increment * self.mean_left_laser_point + self.scan_angle_min # beam between O (rear laserscanner link frame origin) and A (left mirror)
            self.scan_mirrors_angle_right_beam = self.scan_angle_increment * self.mean_right_laser_point + self.scan_angle_min # beam between O (rear laserscanner link frame origin) and B (right mirror)
            self.scan_mirrors_ranges = [self.scan_mirrors_left_beam, self.scan_mirrors_angle_left_beam, self.scan_mirrors_right_beam, self.scan_mirrors_angle_right_beam]
            # sample the angle between the right and left mirrors distances
            self.scan_mirrors_angle_right_left_beam = self.scan_mirrors_angle_left_beam - self.scan_mirrors_angle_right_beam # angle between OA e OB
            self.distance_AB_square = self.scan_mirrors_left_beam ** 2 + self.scan_mirrors_right_beam **2 - 2 * self.scan_mirrors_left_beam * self.scan_mirrors_right_beam * math.cos(self.scan_mirrors_angle_right_left_beam)
            self.distance_AB = math.sqrt(self.distance_AB_square)
            self.cos_angle_OBA = (self.distance_AB ** 2 +  self.scan_mirrors_right_beam ** 2 - self.scan_mirrors_left_beam ** 2) / ( 2 * self.distance_AB * self.scan_mirrors_right_beam)
            
            #rospy.loginfo(self.scan_mirrors_left_beam)
            #rospy.loginfo(self.scan_mirrors_right_beam)
            #rospy.loginfo( self.scan_mirrors_angle_left_beam) 
            #rospy.loginfo(self.scan_mirrors_angle_right_beam)
            #rospy.loginfo(self.scan_mirrors_angle_right_left_beam )
            #rospy.loginfo(self.distance_AB)
            #rospy.logwarn(self.cos_angle_OBA)
           
            self.angle_OBA = math.acos(self.cos_angle_OBA)
            self.angle_correction = (self.scan_mirrors_angle_right_beam - self.angle_OBA)
            rospy.logerr(self.angle_correction)
            # calculates the angle between the x axis on the europallet mirror and the x axis of the rear laserscanner
            #self.angle_correction = -self.scan_angle_increment * (self.samples/2 -(self.mean_left_laser_point + self.mean_right_laser_point)/2)            
            self.qc = quaternion_from_euler(0,0,self.angle_correction - 1.57) # quaternion correction
            #print("correction angle: ",self.angle_correction)
            self.rotation_angle = 1.57 - self.angle_correction
            # print Delta angle
            #rospy.loginfo(self.scan_angle_increment * self.scan_mirrors_index_max)
            # [distance max, angle at that distance [rad], distance min, angle at that distance [rad]]
            #rospy.loginfo("ranges:")
            #rospy.loginfo(self.scan_mirrors_ranges) 
            # rospy.loginfo(self.result)
            self.scan_mirror_result = True
        else: 
            rospy.logwarn("no laserscanner intensieties data found.")

    def frame_listener(self, tf1, tf2):
        (transl, quatrot) = self._listener.lookupTransform(
            tf1, tf2, rospy.Time(0))
        #euler_angles = tf.transformations.euler_from_quaternion(quatrot)
        delta_pose = Pose()
        delta_pose.position.x = transl[0]
        delta_pose.position.y = transl[1]
        delta_pose.position.z = transl[2]
        delta_pose.orientation.x = quatrot[0]
        delta_pose.orientation.y = quatrot[1]
        delta_pose.orientation.z = quatrot[2]
        delta_pose.orientation.w = quatrot[3]
        self.traslation = transl
        #self.euler_angles = euler_angles
        return delta_pose

    def create_ep_frame(self):
        self.scan_mirrors()
        if self.scan_mirror_result:

            # left reflective plate distance
            left_reflective_plate_distance = self.scan_mirrors_ranges[0]
            # right reflective plate distance
            right_reflective_plate_distance = self.scan_mirrors_ranges[2]
            delta_left = self.scan_mirrors_ranges[1]
            delta_right = self.scan_mirrors_ranges[3]
            #print(delta_left, delta_right)

            #       x
            #       ^
            #       |
            #   y<--o  bootprint frame
            # delta_left angle between y and left reflective plate distance
            # delta_right  angle between y and right reflective plate distance
            # left_reflective_plate_distance distance between o and left mirror
            # right_reflective_plate_distance distance between o and right mirror

            self.frame_listener(self._frame_id, self._child_frame_id)
            # rospy.logwarn(trans,quatrot, euler_angles)
            # foot_print_frame
            x_left_mirror = left_reflective_plate_distance * math.cos(delta_left)
            y_left_mirror = left_reflective_plate_distance * math.sin(delta_left)

            # rospy.loginfo("x_left_mirror:")
            # rospy.loginfo(x_left_mirror)
            # rospy.loginfo(" y_left_mirror:")
            # rospy.loginfo(y_left_mirror)

            # foot_print_frame
            x_right_mirror = right_reflective_plate_distance * math.cos(delta_right)
            y_right_mirror = right_reflective_plate_distance * math.sin(delta_right)

            #print("left_reflective_plate_distance: ",left_reflective_plate_distance)
            #print("scan_angle_middle_range_rad - delta_left: ",self.scan_angle_middle_range_rad - delta_left)

            #print("right_reflective_plate_distance: ",right_reflective_plate_distance)
            #print("delta_right - self.scan_angle_middle_range_rad: ",delta_right - self.scan_angle_middle_range_rad)

            # ep_lf europallet left frame coordinates
            self.x_lf = x_left_mirror
            self.y_lf = y_left_mirror
            self.z_lf = self.traslation[2]

            # ep_rf europallet right frame coordinates
            self.x_rf = x_right_mirror
            self.y_rf = y_right_mirror
            self.z_rf = self.traslation[2]

            # ep_cf europallet centre frame coordinates
            self.x_cf = 0.5 * (x_right_mirror + x_left_mirror)
            self.y_cf = 0.5 * (y_right_mirror + y_left_mirror)
            self.z_cf = self.traslation[2]


            self.cf = True

        else:
            self.cf = False

    def pub_ep_frame(self):
        self.create_ep_frame()
        if self.cf:
            #rospy.logwarn("proceding to europallet left frame publishing")
            self._broad_caster_tf.sendTransform((self.x_lf, self.y_lf, self.z_lf),
                                                (self.qc[0], self.qc[1], self.qc[2], self.qc[3]), rospy.Time.now(),
                                                "/ep_lf",
                                                "/rear_laserscanner_link")
            #rospy.logwarn("proceding to europallet right frame publishing")
            self._broad_caster_tf.sendTransform((self.x_rf, self.y_rf, self.z_lf),
                                                (self.qc[0], self.qc[1], self.qc[2], self.qc[3]), rospy.Time.now(),
                                                "/ep_rf",
                                                "/rear_laserscanner_link")

            #rospy.logwarn("proceding to europallet centre frame publishing")
            self._broad_caster_tf.sendTransform((self.x_cf, self.y_cf, self.z_lf),
                                                (self.qc[0], self.qc[1], self.qc[2], self.qc[3]), rospy.Time.now(),
                                                "/ep_cf",
                                                "/rear_laserscanner_link")
            
            #rospy.logwarn("proceding to europallet shifted frame respect to centre frame publishing")
            self._broad_caster_tf.sendTransform((2.5, 0, 0),
                                                (0, 0, 0, 1), rospy.Time.now(),
                                                "/sep_cf",
                                                "/ep_cf")
                                               
            time.sleep(0.1)

            
            # negative value of the delta_pose_position.x means that the amd is at the left of the europallet
            # so that, to centering the amr respect to the europallet,  if the delta_pose_position.x <0 the amr have to turn right and procedes forward.

            delta_pose = Pose() 
            delta_pose_shifted = Pose() 
            delta_pose = self.frame_listener("/ep_cf","/odom") 
            delta_pose_shifted = self.frame_listener("/sep_cf","/odom") 
            #delta_pose.position.x = 0
            #delta_pose.position.y = 0
            #delta_pose.position.z = 0
            #delta_pose.orientation.x = self.qc[0]
            #delta_pose.orientation.y = self.qc[1]
            #delta_pose.orientation.z = self.qc[2]
            #delta_pose.orientation.w = self.qc[3]                
            #rospy.loginfo("delta_pose between ep_cf and base_footprint")
            success = True
            return delta_pose, delta_pose_shifted , success
        else:
            rospy.logerr("it can not get delta_pose")
            success = False 
            delta_pose = Pose() 
            delta_pose.position.x = 0
            delta_pose.position.y = 0
            delta_pose.position.z = 0
            delta_pose.orientation.x = 0
            delta_pose.orientation.y = 0
            delta_pose.orientation.z = 0
            delta_pose.orientation.w = 1
            delta_pose_shifted = delta_pose
            return delta_pose , delta_pose_shifted, success

if __name__ == "__main__":
    rospy.init_node('detect_europallet_node')
    rospy.loginfo('creating detect europallet node')
    detect_cart_obj = DetectEuropallet()    
    rate = rospy.Rate(10)
    control_c = False

    def shutdownhook():
        global control_c
        rospy.loginfo('shutdown script')
        control_c = True

    rospy.on_shutdown(shutdownhook)

    while not control_c:

        detect_cart_obj.pub_ep_frame()

        rate.sleep()