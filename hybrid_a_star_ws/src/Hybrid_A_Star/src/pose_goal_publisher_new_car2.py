#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import scipy.spatial as sp
import os
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from collections import deque
import math

def do_kdtree(array_source, array_dest, k =1):
    '''nearest neighbor kdtree, fast because creates a binary octatree
    Args: 
        1) array_source: array with more elements in whom we want to find nearest neighbour
        2) array_dest: array with less elements whose nearest neighbour we need to find
    Returns:
        1) indexes: arrray having all indexes
        2) dist: array having distances
    '''
    mytree = sp.cKDTree(array_source)
    dist, indexes = mytree.query(array_dest, k)
    return indexes, dist

class GetGoal:
    def __init__(self):
        self.look_ahead_index = 5
        # self.look_ahead_index = 3
        self.error_buffer = 10
        scale_100 = True
        self.pos_obstacles =[]
        
        if (scale_100 == True):
            # waypoints_name = "waypoints1_100scale.npy" #Nov2 manual Ronit
            # for 100 scale maps Nov2
            # self.translate_x = -23.433744557914416
            # self.translate_y = 37.368772684946485
            waypoints_name = "Nov10_manual_jash_100scale2.npy"
            self.translate_x = -17.964026958248915 #-32.964026958248915
            self.translate_y = 35.39230053680539
#             x_translation = -3.2964026958248915
# y_translation = 3.539230053680539

            # self.translate_x = -20.41996779977085
            # self.translate_y = 39.681654685361023
            self.scale_factor = 100
        # for 10 scale maps Nov2
        else:
            # waypoints_name = "waypoints1_10scale.npy"
            waypoints_name = "Nov10_manual_jash_100scale2.npy"
            self.translate_x = -3.2964026958248915
            self.translate_y = 3.539230053680539

            # self.translate_x = -2.041996779977085
            # self.translate_y = 3.9681654685361023
            # self.translate_x = -2.3433744557914416
            # self.translate_y = 3.7368772684946485
            self.scale_factor = 10
        self.waypoints = np.load(waypoints_name, allow_pickle=True)
        rospy.init_node('publish_curr_pose_and_goal_pose_new_car2')
        # rospy.Subscriber('/car2/fused_nucklie', Odometry, self.odom_callback)
        # rospy.Subscriber('/debug_pose2', Odometry, self.odom_callback)
        # rospy.Subscriber('/car2/fused_nucklie', Odometry, self.odom_callback)
        # rospy.Subscriber('/car1/fused', Odometry, self.odom_callback)
        rospy.Subscriber('/car2/fused', Odometry, self.odom_callback)
        # rospy.Subscriber('/debug_pose2', Odometry, self.odom_callback)
        # rospy.Subscriber('/car1/fused', Odometry, self.odom_callback)
        rospy.Subscriber('/rccar_pose_new', Float32MultiArray, self.obstacle_callback)
        # self.pose_cov_publisher = rospy.Publisher('/car2/run_hybrid_astar/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
        # self.goal_publisher = rospy.Publisher('/car2/run_hybrid_astar/planner_goal_pos', PoseStamped, queue_size=10)
        # self.pose_cov_publisher = rospy.Publisher('/car_1/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
        # self.goal_publisher = rospy.Publisher('/car_1/planner_goal_pos', PoseStamped, queue_size=10)
        self.pose_cov_publisher = rospy.Publisher('/car_2/planner_curr_pos_new', PoseWithCovarianceStamped, queue_size=10)
        self.goal_publisher = rospy.Publisher('/car_2/planner_goal_pos_new', PoseStamped, queue_size=10)
        # self.pose_cov_publisher = rospy.Publisher('/car_1/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
        # self.goal_publisher = rospy.Publisher('/car_1/planner_goal_pos', PoseStamped, queue_size=10)
        # self.goal_id_dq = deque(maxlen=1)
        
        
        
        
    def get_goal_for_pose(self, curr):
        # near_id, _ = do_kdtree(self.waypoints[:,:-1], curr, k =1)
        near_id, _ = do_kdtree(self.waypoints, curr, k =1)
        goal_id = near_id + self.look_ahead_index
        flag_cond = False
        if (len(self.pos_obstacles) !=0):
            for i in range(len(self.pos_obstacles)):
                id_obs = self.pos_obstacles[i][0]
                x_obs = self.pos_obstacles[i][1]
                y_obs = self.pos_obstacles[i][2]
                if id_obs <1000:
                    obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                    near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                    print("###########################################  ", near_id_obs, "  #############################################")
                    if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>0 and near_id_obs<self.look_ahead_index:
                        goal_id = near_id_obs -2 
                        flag_cond = True
                        print("triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
                    if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>near_id:
                        goal_id = near_id_obs -2 
                        flag_cond = True
                        print("triggggeeerrrreeedddddd nooooooooooooooooohhhhhhhhhhhhhere")
                    if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs==0:
                        goal_id = self.waypoints.shape[0]-2
                        flag_cond = True
                        print("triggggeeerrrreeedddddd tttttttttttthere")
                    if near_id_obs<goal_id and near_id_obs> near_id:
                        goal_id = near_id_obs-2
                        flag_cond = True
                        print("triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                if id_obs <1000:
                    obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                    near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                    print("###########################################  ", near_id_obs, "  #############################################")
                    if goal_id>near_id_obs-1 and goal_id< near_id_obs+1:
                        goal_id = goal_id+3
                        print("triggggeerrrreeeedddddddddddddddddd ahhhhhheeeeaaaadddddddddddddddddd")





        # if (len(self.goal_id_dq) ==0):
        #     self.goal_id_dq.append(goal_id)
            
        # if(goal_id <= self.waypoints.shape[0]-1 and abs(goal_id-self.goal_id_dq[0])>self.error_buffer and flag_cond == False):
        #     print("triggeredddd########################################")
        #     goal_id = np.asarray([self.goal_id_dq[0]])[0]


        if (goal_id > self.waypoints.shape[0]-1):
            goal_id = goal_id - self.waypoints.shape[0]
        # self.goal_id_dq.append(goal_id)
        goal_pose = self.waypoints[goal_id]

        print(near_id, "                ", goal_id, type(goal_id))
        # if (len(self.pos_obstacles) !=0):
        #     for i in range(len(self.pos_obstacles)):
        #         x_obs = self.pos_obstacles[i][0]
        #         y_obs = self.pos_obstacles[i][1]
        #         # obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
        #         # near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
        #         # if near_id_obs<goal_id

                
                
                
                
                
        #         # dist = math.sqrt(((goal_pose[0,0]*self.scale_factor + self.translate_x)-x_obs)**2 + ((goal_pose[0,1]*self.scale_factor + self.translate_y)-y_obs)**2)
        #         # if (dist < 2):
                    
        #         #     goal_id = goal_id+1
        #         #     print("inside obstacle", near_id, "                ", goal_id, type(goal_id))
        #         #     self.goal_id_dq.append(goal_id)
        #         #     goal_pose = self.waypoints[goal_id]
        
        return goal_pose, goal_id
    
    def get_obstacle_pose(self, msg, scale_factor, translate_x, translate_y ):
    
        # msg_ts_gps = msg.header.stamp.to_sec()
        pose = msg.data
        # pose_rccar = np.asarray(msg.data)
        length_msg = len(pose)
        i =0
        pos_obs = []
        while (i<length_msg):
            
            car_id =  pose[i]
            pos_x   = pose[i+1]#*scale_factor +translate_x
            pos_y   = pose[i+2]#*scale_factor +translate_y
            pos_v   = pose[i+3]
            pos_yaw = pose[i+4]
            if (car_id<1000 and pos_v <=0.05):
                pos_obs.append([car_id, pos_x, pos_y])
            i=i+5
        
        return pos_obs
    
    def obstacle_callback(self,msg):
        self.pos_obstacles = self.get_obstacle_pose(msg, self.scale_factor, self.translate_x, self.translate_y )

    def odom_callback(self,msg):
        # Create a PoseStamped message
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header.stamp = msg.header.stamp
        pose_cov_msg.header.frame_id = "world"

        pose_cov_msg.pose.pose.position.x = msg.pose.pose.position.x*self.scale_factor + self.translate_x
        pose_cov_msg.pose.pose.position.y = msg.pose.pose.position.y*self.scale_factor + self.translate_y
        pose_cov_msg.pose.pose.position.z = 0.0
        pose_cov_msg.pose.pose.orientation = msg.pose.pose.orientation


        pose_cov_msg.pose.covariance = msg.pose.covariance

        self.pose_cov_publisher.publish(pose_cov_msg)
        
        roll,pitch,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # print(yaw, type(yaw))
        self.current_pose = np.asarray([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]).reshape(1,-1)
        self.goal_pose_val, gola_id = self.get_goal_for_pose(self.current_pose)
        # print(self.current_pose,    "                  ",        self.goal_pose_val)
        # print(self.goal_pose_val.shape)
        

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = msg.header.stamp
        pose_stamped_msg.header.frame_id = "world"

        pose_stamped_msg.pose.position.x = self.goal_pose_val[0,0]*self.scale_factor + self.translate_x
        pose_stamped_msg.pose.position.y = self.goal_pose_val[0,1]*self.scale_factor + self.translate_y
        pose_stamped_msg.pose.position.z = 0.0
        # if (len(self.pos_obstacles) !=0):
        #     for i in range(len(self.pos_obstacles)):
        #         x_obs = self.pos_obstacles[i][0]
        #         y_obs = self.pos_obstacles[i][1]
        #         dist = math.sqrt((pose_stamped_msg.pose.position.x-x_obs)**2 + (pose_stamped_msg.pose.position.y-y_obs)**2)
        #         if (dist < 2):
        #             print("inside obstacle")
        #             gola_id = gola_id+1
        #             goal_pose_val_new = self.waypoints[gola_id]
        #             pose_stamped_msg.pose.position.x = goal_pose_val_new[0,0]*self.scale_factor + self.translate_x
        #             pose_stamped_msg.pose.position.y = goal_pose_val_new[0,1]*self.scale_factor + self.translate_y



        # Convert theta to a quaternion
        x, y, z, w = quaternion_from_euler(0, 0, self.goal_pose_val[0,2])  # Convert theta to quaternion

        # Set the orientation of PoseStamped message
        pose_stamped_msg.pose.orientation.x = x
        pose_stamped_msg.pose.orientation.y = y
        pose_stamped_msg.pose.orientation.z = z
        pose_stamped_msg.pose.orientation.w = w

        self.goal_publisher.publish(pose_stamped_msg)



    def main(self):
        rospy.spin()

if __name__ == '__main__':
    get_goal = GetGoal()
    get_goal.main()