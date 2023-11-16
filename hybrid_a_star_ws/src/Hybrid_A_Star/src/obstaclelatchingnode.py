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

class ObsLatch:
    def __init__(self):
        rospy.init_node('obstacle_latching')
        rospy.Subscriber('/rccar_pose', Float32MultiArray, self.obstacle_callback)
        self.latched_obs_pose = rospy.Publisher('/rccar_pose', Float32MultiArray, queue_size=10)
        self.global_pose_obstacles =deque(maxlen=5)
        self.global_car_id_list =deque(maxlen=5)
        self.counter = 0
    
    def get_obstacle_pose(self, msg, scale_factor, translate_x, translate_y ):
    
        # msg_ts_gps = msg.header.stamp.to_sec()
        pose = msg.data
        # pose_rccar = np.asarray(msg.data)
        length_msg = len(pose)
        i =0
        pos_obs = []
        obs_id =[]
        while (i<length_msg):
            
            car_id =  pose[i]
            pos_x   = pose[i+1]#*scale_factor +translate_x
            pos_y   = pose[i+2]#*scale_factor +translate_y
            pos_v   = pose[i+3]
            pos_yaw = pose[i+4]
            if (car_id<1000 and pos_v <=0.05):
                pos_obs.append(pos_x, pos_y)
                obs_id.append(car_id)
                
            i=i+5
        
        return pos_obs, obs_id
    
    def obstacle_callback(self,msg):
        pos_obstacles, car_ids = self.get_obstacle_pose(msg, self.scale_factor, self.translate_x, self.translate_y )
        if len(self.global_car_id_list < 6):
            self.global_car_id_list.append(car_ids)
            self.global_pose_obstacles.append(pos_obstacles)
        else:    
            for id in car_ids:
                presence_checks = [id in sublist for sublist in self.global_car_id_list]
                if all(not check for check in presence_checks):
                    print("now drop the object")
                    # publish x y current value



                else:
                    print("do not drop the obstacle ")


            self.global_car_id_list.append(car_ids)
            self.global_pose_obstacles.append(pos_obstacles)

        self.counter = self.counter +1

    # def odom_callback(self,msg):
    #     # Create a PoseStamped message
    #     pose_cov_msg = PoseWithCovarianceStamped()
    #     pose_cov_msg.header.stamp = msg.header.stamp
    #     pose_cov_msg.header.frame_id = "world"

    #     pose_cov_msg.pose.pose.position.x = msg.pose.pose.position.x*self.scale_factor + self.translate_x
    #     pose_cov_msg.pose.pose.position.y = msg.pose.pose.position.y*self.scale_factor + self.translate_y
    #     pose_cov_msg.pose.pose.position.z = 0.0
    #     pose_cov_msg.pose.pose.orientation = msg.pose.pose.orientation


    #     pose_cov_msg.pose.covariance = msg.pose.covariance

    #     self.pose_cov_publisher.publish(pose_cov_msg)
    #     self.current_pose = np.asarray([msg.pose.pose.position.x, msg.pose.pose.position.y]).reshape(1,-1)
    #     self.goal_pose_val, gola_id = self.get_goal_for_pose(self.current_pose)
    #     # print(self.current_pose,    "                  ",        self.goal_pose_val)
    #     # print(self.goal_pose_val.shape)
        

    #     pose_stamped_msg = PoseStamped()
    #     pose_stamped_msg.header.stamp = msg.header.stamp
    #     pose_stamped_msg.header.frame_id = "world"

    #     pose_stamped_msg.pose.position.x = self.goal_pose_val[0,0]*self.scale_factor + self.translate_x
    #     pose_stamped_msg.pose.position.y = self.goal_pose_val[0,1]*self.scale_factor + self.translate_y
    #     pose_stamped_msg.pose.position.z = 0.0
    #     # if (len(self.pos_obstacles) !=0):
    #     #     for i in range(len(self.pos_obstacles)):
    #     #         x_obs = self.pos_obstacles[i][0]
    #     #         y_obs = self.pos_obstacles[i][1]
    #     #         dist = math.sqrt((pose_stamped_msg.pose.position.x-x_obs)**2 + (pose_stamped_msg.pose.position.y-y_obs)**2)
    #     #         if (dist < 2):
    #     #             print("inside obstacle")
    #     #             gola_id = gola_id+1
    #     #             goal_pose_val_new = self.waypoints[gola_id]
    #     #             pose_stamped_msg.pose.position.x = goal_pose_val_new[0,0]*self.scale_factor + self.translate_x
    #     #             pose_stamped_msg.pose.position.y = goal_pose_val_new[0,1]*self.scale_factor + self.translate_y



    #     # Convert theta to a quaternion
    #     x, y, z, w = quaternion_from_euler(0, 0, self.goal_pose_val[0,2])  # Convert theta to quaternion

    #     # Set the orientation of PoseStamped message
    #     pose_stamped_msg.pose.orientation.x = x
    #     pose_stamped_msg.pose.orientation.y = y
    #     pose_stamped_msg.pose.orientation.z = z
    #     pose_stamped_msg.pose.orientation.w = w

    #     self.goal_publisher.publish(pose_stamped_msg)



    def main(self):
        rospy.spin()

if __name__ == '__main__':
    get_goal = ObsLatch()
    get_goal.main()