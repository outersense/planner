import rospy
from std_msgs.msg import Float32MultiArray
import scipy.spatial as sp
import os
import numpy as np
import matplotlib.pyplot as plt
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
        self.latched_obs_pose = rospy.Publisher('/rccar_pose_new2', Float32MultiArray, queue_size=10)
        self.global_pose_obstacles =deque(maxlen=5)
        self.global_car_id_list =deque(maxlen=5)
        self.counter = 1
    
    def get_obstacle_pose(self, msg):
    
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
                pos_obs.append(pos_x)
                pos_obs.append(pos_y)
                obs_id.append(car_id)
                
            i=i+5
        
        return pos_obs, obs_id
    
    def obstacle_callback(self,msg):
        publish_list = []
        pos_obstacles, car_ids = self.get_obstacle_pose(msg)
        print("@####################################  ", len(car_ids), "@###############################################")
        if (len(self.global_car_id_list) < 5 and len(car_ids)==0):
            #  publish something with velocity
            print("no obstacle no latching chill scenes")
            publish_list.append(0)
            publish_list.append(0)
            publish_list.append(0)
            publish_list.append(1)
            publish_list.append(0)
            print(publish_list)
            self.latched_obs_pose.publish(Float32MultiArray(data=publish_list))

        elif (len(self.global_car_id_list) < 5 and len(car_ids)!=0):
            #  make the global lists and publish immediately for these counts
            print("yes obstacle just added")
            self.global_car_id_list.append(car_ids)
            self.global_pose_obstacles.append(pos_obstacles)
            for obslist in self.global_pose_obstacles:
                i =0
                while i < len(obslist):
                    publish_list.append(0)
                    publish_list.append(i)
                    publish_list.append(i+1)
                    publish_list.append(0)
                    publish_list.append(0)
                    i = i+2
            self.latched_obs_pose.publish(Float32MultiArray(data=publish_list))



        elif(len(self.global_car_id_list) >= 5 and len(car_ids)==0 and self.counter%10 != 0):
            print("latching and flickering")
            self.counter = self.counter+1
            for obslist in self.global_pose_obstacles:
                i =0
                while i < len(obslist):
                    publish_list.append(0)
                    publish_list.append(i)
                    publish_list.append(i+1)
                    publish_list.append(0)
                    publish_list.append(0)
                    i = i+2
            self.latched_obs_pose.publish(Float32MultiArray(data=publish_list))

        elif (len(self.global_car_id_list) >= 5 and len(car_ids)==0 and self.counter %10 ==0):
            print("clearing no obstacle")
            self.counter = 1
            publish_list.append(0)
            publish_list.append(0)
            publish_list.append(0)
            publish_list.append(1)
            publish_list.append(0)
            self.latched_obs_pose.publish(Float32MultiArray(data=publish_list))
            self.global_car_id_list = []
            self.global_pose_obstacles = []
            

        else:   # (len(self.global_car_id_list >= 6) and len(car_ids)!=0 )
            print("yes obstcale ")
            self.global_car_id_list.append(car_ids)
            self.global_pose_obstacles.append(pos_obstacles)  
            for obslist in self.global_pose_obstacles:
                i =0
                while i < len(obslist):
                    publish_list.append(0)
                    publish_list.append(i)
                    publish_list.append(i+1)
                    publish_list.append(0)
                    publish_list.append(0)
                    i = i+2
            self.latched_obs_pose.publish(Float32MultiArray(data=publish_list))


            # for id in car_ids:
            #     presence_checks = [id in sublist for sublist in self.global_car_id_list]
            #     if all(not check for check in presence_checks):
            #         print("now drop the object")
            #         # publish x y current value
            #         for obslist in self.global_pose_obstacles:
            #             while i < len(obslist):
            #                 publish_list.append(0)
            #                 publish_list.append(i)
            #                 publish_list.append(i+1)
            #                 publish_list.append(0)
            #                 publish_list.append(0)
            #                 i = i+2
            #         self.latched_obs_pose.publish(publish_list)



            #     else:
            #         print("do not drop the obstacle ")
            #         # send all in this deque
            #         for obslist in self.global_pose_obstacles:
            #             while i < len(obslist):
            #                 publish_list.append(0)
            #                 publish_list.append(i)
            #                 publish_list.append(i+1)
            #                 publish_list.append(0)
            #                 publish_list.append(0)
            #                 i = i+2
            #         self.latched_obs_pose.publish(publish_list)




            

        

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    get_goal = ObsLatch()
    get_goal.main()