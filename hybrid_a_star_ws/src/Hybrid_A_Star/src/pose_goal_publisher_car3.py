# #!/usr/bin/env python3


# #  This is node that listens to fused data that is of pose stampped message type
# # header: 
# #   seq: 2409
# #   stamp: 
# #     secs: 1697571825
# #     nsecs:  84017754
# #   frame_id: "world"
# # child_frame_id: "car2/base_link"
# # pose: 
# #   pose: 
# #     position: 
# #       x: 4.81719369974616
# #       y: 0.20964775495888713
# #       z: 8.515506985626891e-17
# #     orientation: 
# #       x: -1.0645422195947236e-18
# #       y: 1.1655235856498888e-16
# #       z: 0.9891147136329634
# #       w: 0.1471464687812136
# #   covariance: [0.10555161138815394, 3.062115609911795e-09, 2.7355735951529573e-25, -4.650460029946241e-25, 3.83505836792795e-21, -1.6506096026961142e-14, 3.062115609912007e-09, 0.10555163843494872, 4.385836841402432e-24, -1.4945889212464688e-24, -1.1017906097304037e-21, -5.33312875232135e-14, 2.7355735950890085e-25, -2.231810010516407e-24, 1.5558031201453346e-06, -2.1008079361714568e-19, -7.917988555775595e-15, 1.663223160174368e-21, 1.2769843904344625e-23, -1.4946895457664422e-24, -2.1008079361714366e-19, 1.4066593508457507e-06, 6.383942135102298e-17, 1.1471824004974714e-32, 3.835058367821831e-21, -1.1050993325303647e-21, -7.917988562393027e-15, 6.383942135102298e-17, 1.406659350783465e-06, -1.2472962453739365e-21, -1.6506096028615503e-14, -5.33312875232135e-14, 1.6632231601754195e-21, -2.1488570665025156e-29, -1.247296223872762e-21, 3.4066173070679447e-06]
# # twist: 
# #   twist: 
# #     linear: 
# #       x: 0.49894534218037734
# #       y: -1.0973387133042312e-10
# #       z: -8.46226250344752e-17
# #     angular: 
# #       x: -9.33038540958811e-18
# #       y: -8.544765695008065e-17
# #       z: -0.0033367921510789583
# #   covariance: [0.0005010088481122983, -1.5796431104794825e-21, 6.78012999858353e-26, -1.7462633274307452e-28, -1.9776938887050552e-27, 5.988393154936348e-22, -1.5796431104794818e-21, 0.0005010088481119932, 5.0040238497350123e-29, 4.210623271354027e-33, 1.7720143016089391e-31, 2.553776130835845e-22, -3.559312113967227e-26, 4.9251377592134526e-29, 1.4663689031948455e-06, 6.308286649009168e-23, -9.854010218034441e-19, 1.7570132866342285e-30, -1.746263327430745e-28, 4.210623271353815e-33, 6.308286649009173e-23, 1.2347785881310081e-06, 1.2296771546825321e-24, 4.086813419138285e-28, -1.977693912779225e-27, 1.7717735603371286e-31, -9.854010218539304e-19, 1.2296771546825329e-24, 1.2347785881309926e-06, -2.8090322806766235e-29, 5.988910143312232e-22, 2.5537761352731877e-22, 1.7816651626233804e-30, 4.1951093051238335e-30, -2.828753807615544e-29, 0.0004002122586454154]
# # ---

# #  it gets the car2 values x and y and publishes in format initial
# # header: 
# #   seq: 0
# #   stamp: 
# #     secs: 1697571817
# #     nsecs: 344560695
# #   frame_id: "world"
# # pose: 
# #   pose: 
# #     position: 
# #       x: 84.54855346679688
# #       y: 29.91866683959961
# #       z: 0.0
# #     orientation: 
# #       x: 0.0
# #       y: 0.0
# #       z: 0.18952627488212456
# #       w: 0.9818756495245746
# #   covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
# # ---

# #  once obtained xy and theta it checks for nearest neighbour and them finds a goal states xy and theta and publishes it in the format
# # and goal as
# # header: 
# #   seq: 1
# #   stamp: 
# #     secs: 1697571823
# #     nsecs: 370301804
# #   frame_id: "world"
# # pose: 
# #   position: 
# #     x: 105.09276580810547
# #     y: 46.70957946777344
# #     z: 0.0
# #   orientation: 
# #     x: 0.0
# #     y: 0.0
# #     z: 0.38412213944847984
# #     w: 0.9232822872694584
# # ---


# import rospy
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import PoseStamped
# from geometry_msgs.msg import PoseWithCovarianceStamped
# import scipy.spatial as sp
# import os
# import numpy as np
# import matplotlib.pyplot as plt
# from tf.transformations import quaternion_from_euler
# from collections import deque
# import math

# def do_kdtree(array_source, array_dest, k =1):
#     '''nearest neighbor kdtree, fast because creates a binary octatree
#     Args: 
#         1) array_source: array with more elements in whom we want to find nearest neighbour
#         2) array_dest: array with less elements whose nearest neighbour we need to find
#     Returns:
#         1) indexes: arrray having all indexes
#         2) dist: array having distances
#     '''
#     mytree = sp.cKDTree(array_source)
#     dist, indexes = mytree.query(array_dest, k)
#     return indexes, dist

# class GetGoal:
#     def __init__(self):
#         self.look_ahead_index = 5
#         # self.look_ahead_index = 3
#         self.error_buffer = 10
#         scale_100 = True
#         self.pos_obstacles =[]
        
#         if (scale_100 == True):
#             # waypoints_name = "waypoints1_100scale.npy" #Nov2 manual Ronit
#             # for 100 scale maps Nov2
#             # self.translate_x = -23.433744557914416
#             # self.translate_y = 37.368772684946485
#             waypoints_name = "Nov10_manual_jash_100scale2.npy"
#             self.translate_x = -17.964026958248915 #-32.964026958248915
#             self.translate_y = 35.39230053680539
# #             x_translation = -3.2964026958248915
# # y_translation = 3.539230053680539

#             # self.translate_x = -20.41996779977085
#             # self.translate_y = 39.681654685361023
#             self.scale_factor = 100
#         # for 10 scale maps Nov2
#         else:
#             # waypoints_name = "waypoints1_10scale.npy"
#             waypoints_name = "Nov10_manual_jash_100scale2.npy"
#             self.translate_x = -3.2964026958248915
#             self.translate_y = 3.539230053680539

#             # self.translate_x = -2.041996779977085
#             # self.translate_y = 3.9681654685361023
#             # self.translate_x = -2.3433744557914416
#             # self.translate_y = 3.7368772684946485
#             self.scale_factor = 10
#         self.waypoints = np.load(waypoints_name, allow_pickle=True)
#         rospy.init_node('publish_curr_pose_and_goal_pose_car2')
#         # rospy.Subscriber('/car2/fused_nucklie', Odometry, self.odom_callback)
#         # rospy.Subscriber('/debug_pose2', Odometry, self.odom_callback)
#         # rospy.Subscriber('/car2/fused_nucklie', Odometry, self.odom_callback)
#         # rospy.Subscriber('/car1/fused', Odometry, self.odom_callback)
#         rospy.Subscriber('/car2/fused', Odometry, self.odom_callback)
#         # rospy.Subscriber('/debug_pose2', Odometry, self.odom_callback)
#         # rospy.Subscriber('/car1/fused', Odometry, self.odom_callback)
#         rospy.Subscriber('/rccar_pose_new', Float32MultiArray, self.obstacle_callback)
#         # self.pose_cov_publisher = rospy.Publisher('/car2/run_hybrid_astar/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
#         # self.goal_publisher = rospy.Publisher('/car2/run_hybrid_astar/planner_goal_pos', PoseStamped, queue_size=10)
#         # self.pose_cov_publisher = rospy.Publisher('/car_1/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
#         # self.goal_publisher = rospy.Publisher('/car_1/planner_goal_pos', PoseStamped, queue_size=10)
#         self.pose_cov_publisher = rospy.Publisher('/car_2/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
#         self.goal_publisher = rospy.Publisher('/car_2/planner_goal_pos', PoseStamped, queue_size=10)
#         # self.pose_cov_publisher = rospy.Publisher('/car_1/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
#         # self.goal_publisher = rospy.Publisher('/car_1/planner_goal_pos', PoseStamped, queue_size=10)
#         self.goal_id_dq = deque(maxlen=1)
        
        
        
        
#     def get_goal_for_pose(self, curr):
#         near_id, _ = do_kdtree(self.waypoints[:,:-1], curr, k =1)
#         goal_id = near_id + self.look_ahead_index
#         flag_cond = False
#         if (len(self.pos_obstacles) !=0):
#             for i in range(len(self.pos_obstacles)):
#                 x_obs = self.pos_obstacles[i][0]
#                 y_obs = self.pos_obstacles[i][1]
#                 obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
#                 near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
#                 print("###########################################  ", near_id_obs, "  #############################################")
#                 if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>0 and near_id_obs<self.look_ahead_index:
#                     goal_id = near_id_obs -2 
#                     flag_cond = True
#                     print("triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
#                 if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs==0:
#                     goal_id = self.waypoints.shape[0]-2
#                     flag_cond = True
#                     print("triggggeeerrrreeedddddd tttttttttttthere")
#                 if near_id_obs<goal_id and near_id_obs> near_id:
#                     goal_id = near_id_obs-2
#                     flag_cond = True
#                     print("triggggeeerrrreeedddddd wwwwwwwwwwwwhere")



#         if (len(self.goal_id_dq) ==0):
#             self.goal_id_dq.append(goal_id)
            
#         if(goal_id <= self.waypoints.shape[0]-1 and abs(goal_id-self.goal_id_dq[0])>self.error_buffer and flag_cond == False):
#             print("triggeredddd########################################")
#             goal_id = np.asarray([self.goal_id_dq[0]])[0]


#         if (goal_id > self.waypoints.shape[0]-1):
#             goal_id = goal_id - self.waypoints.shape[0]
#         self.goal_id_dq.append(goal_id)
#         goal_pose = self.waypoints[goal_id]

#         print(near_id, "                ", goal_id, type(goal_id))
#         # if (len(self.pos_obstacles) !=0):
#         #     for i in range(len(self.pos_obstacles)):
#         #         x_obs = self.pos_obstacles[i][0]
#         #         y_obs = self.pos_obstacles[i][1]
#         #         # obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
#         #         # near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
#         #         # if near_id_obs<goal_id

                
                
                
                
                
#         #         # dist = math.sqrt(((goal_pose[0,0]*self.scale_factor + self.translate_x)-x_obs)**2 + ((goal_pose[0,1]*self.scale_factor + self.translate_y)-y_obs)**2)
#         #         # if (dist < 2):
                    
#         #         #     goal_id = goal_id+1
#         #         #     print("inside obstacle", near_id, "                ", goal_id, type(goal_id))
#         #         #     self.goal_id_dq.append(goal_id)
#         #         #     goal_pose = self.waypoints[goal_id]
        
#         return goal_pose, goal_id
    
#     def get_obstacle_pose(self, msg, scale_factor, translate_x, translate_y ):
    
#         # msg_ts_gps = msg.header.stamp.to_sec()
#         pose = msg.data
#         # pose_rccar = np.asarray(msg.data)
#         length_msg = len(pose)
#         i =0
#         pos_obs = []
#         while (i<length_msg):
            
#             car_id =  pose[i]
#             pos_x   = pose[i+1]#*scale_factor +translate_x
#             pos_y   = pose[i+2]#*scale_factor +translate_y
#             pos_v   = pose[i+3]
#             pos_yaw = pose[i+4]
#             if (car_id<1000 and pos_v <=0.05):
#                 pos_obs.append([pos_x, pos_y])
#             i=i+5
        
#         return pos_obs
    
#     def obstacle_callback(self,msg):
#         self.pos_obstacles = self.get_obstacle_pose(msg, self.scale_factor, self.translate_x, self.translate_y )

#     def odom_callback(self,msg):
#         # Create a PoseStamped message
#         pose_cov_msg = PoseWithCovarianceStamped()
#         pose_cov_msg.header.stamp = msg.header.stamp
#         pose_cov_msg.header.frame_id = "world"

#         pose_cov_msg.pose.pose.position.x = msg.pose.pose.position.x*self.scale_factor + self.translate_x
#         pose_cov_msg.pose.pose.position.y = msg.pose.pose.position.y*self.scale_factor + self.translate_y
#         pose_cov_msg.pose.pose.position.z = 0.0
#         pose_cov_msg.pose.pose.orientation = msg.pose.pose.orientation


#         pose_cov_msg.pose.covariance = msg.pose.covariance

#         self.pose_cov_publisher.publish(pose_cov_msg)
#         self.current_pose = np.asarray([msg.pose.pose.position.x, msg.pose.pose.position.y]).reshape(1,-1)
#         self.goal_pose_val, gola_id = self.get_goal_for_pose(self.current_pose)
#         # print(self.current_pose,    "                  ",        self.goal_pose_val)
#         # print(self.goal_pose_val.shape)
        

#         pose_stamped_msg = PoseStamped()
#         pose_stamped_msg.header.stamp = msg.header.stamp
#         pose_stamped_msg.header.frame_id = "world"

#         pose_stamped_msg.pose.position.x = self.goal_pose_val[0,0]*self.scale_factor + self.translate_x
#         pose_stamped_msg.pose.position.y = self.goal_pose_val[0,1]*self.scale_factor + self.translate_y
#         pose_stamped_msg.pose.position.z = 0.0
#         # if (len(self.pos_obstacles) !=0):
#         #     for i in range(len(self.pos_obstacles)):
#         #         x_obs = self.pos_obstacles[i][0]
#         #         y_obs = self.pos_obstacles[i][1]
#         #         dist = math.sqrt((pose_stamped_msg.pose.position.x-x_obs)**2 + (pose_stamped_msg.pose.position.y-y_obs)**2)
#         #         if (dist < 2):
#         #             print("inside obstacle")
#         #             gola_id = gola_id+1
#         #             goal_pose_val_new = self.waypoints[gola_id]
#         #             pose_stamped_msg.pose.position.x = goal_pose_val_new[0,0]*self.scale_factor + self.translate_x
#         #             pose_stamped_msg.pose.position.y = goal_pose_val_new[0,1]*self.scale_factor + self.translate_y



#         # Convert theta to a quaternion
#         x, y, z, w = quaternion_from_euler(0, 0, self.goal_pose_val[0,2])  # Convert theta to quaternion

#         # Set the orientation of PoseStamped message
#         pose_stamped_msg.pose.orientation.x = x
#         pose_stamped_msg.pose.orientation.y = y
#         pose_stamped_msg.pose.orientation.z = z
#         pose_stamped_msg.pose.orientation.w = w

#         self.goal_publisher.publish(pose_stamped_msg)



#     def main(self):
#         rospy.spin()

# if __name__ == '__main__':
#     get_goal = GetGoal()
#     get_goal.main()


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
        self.look_ahead_index = 6
        # self.look_ahead_index = 3
        self.error_buffer = 10
        scale_100 = True
        self.pos_obstacles =[]
        self.mybigdict = {}
        self.countercheck = 0

        
        
        if (scale_100 == True):
            # waypoints_name = "waypoints1_100scale.npy" #Nov2 manual Ronit
            # for 100 scale maps Nov2
            # self.translate_x = -23.433744557914416
            # self.translate_y = 37.368772684946485
            waypoints_name = "/home/dhanesh/Masters/OuterSense/Planning_100scale/planner/hybrid_a_star_ws/src/Hybrid_A_Star/src/Nov10_manual_jash_100scale2.npy"
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
            waypoints_name = "/home/dhanesh/Masters/OuterSense/Planning_100scale/planner/hybrid_a_star_ws/src/Hybrid_A_Star/src/Nov10_manual_jash_100scale2.npy"
            self.translate_x = -3.2964026958248915
            self.translate_y = 3.539230053680539

            # self.translate_x = -2.041996779977085
            # self.translate_y = 3.9681654685361023
            # self.translate_x = -2.3433744557914416
            # self.translate_y = 3.7368772684946485
            self.scale_factor = 10
        self.waypoints = np.load(waypoints_name, allow_pickle=True)
        rospy.init_node('publish_curr_pose_and_goal_pose_car3')
        # rospy.Subscriber('/car2/fused_nucklie', Odometry, self.odom_callback)
        # rospy.Subscriber('/debug_pose2', Odometry, self.odom_callback)
        # rospy.Subscriber('/car2/fused_nucklie', Odometry, self.odom_callback)
        # rospy.Subscriber('/car1/fused', Odometry, self.odom_callback)
        rospy.Subscriber('/car3/fused', Odometry, self.odom_callback)
        # rospy.Subscriber('/debug_pose2', Odometry, self.odom_callback)
        # rospy.Subscriber('/car1/fused', Odometry, self.odom_callback)
        # rospy.Subscriber('/rccar_pose_new', Float32MultiArray, self.obstacle_callback)
        rospy.Subscriber('/rccar_pose', Float32MultiArray, self.obstacle_callback)
        rospy.Subscriber('/perception_outage', Float32MultiArray, self.perception_loss_callback)
        # self.pose_cov_publisher = rospy.Publisher('/car2/run_hybrid_astar/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
        # self.goal_publisher = rospy.Publisher('/car2/run_hybrid_astar/planner_goal_pos', PoseStamped, queue_size=10)
        # self.pose_cov_publisher = rospy.Publisher('/car_1/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
        # self.goal_publisher = rospy.Publisher('/car_1/planner_goal_pos', PoseStamped, queue_size=10)
        self.pose_cov_publisher = rospy.Publisher('/car_3/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
        self.goal_publisher = rospy.Publisher('/car_3/planner_goal_pos', PoseStamped, queue_size=10)
        # self.pose_cov_publisher = rospy.Publisher('/car_1/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)
        # self.goal_publisher = rospy.Publisher('/car_1/planner_goal_pos', PoseStamped, queue_size=10)
        # self.goal_id_dq = deque(maxlen=1)
        
        
        
        
    def get_goal_for_pose(self, curr):
        list_of_ids = []
        # near_id, _ = do_kdtree(self.waypoints[:,:-1], curr, k =1)
        if curr[0][-1]<0 and curr[0][0]>4 and curr[0][0]<1:
            curr[0][-1]= curr[0][-1]+2*3.14
        near_id, _ = do_kdtree(self.waypoints, curr, k =1)
        # print(self.waypoints)
        # print(curr)
        # print("yeh 16 k lia",near_id, " " ,_)
        goal_id = near_id + self.look_ahead_index
        flag_cond = False
        
        if (len(self.pos_obstacles) !=0):
            for i in range(len(self.pos_obstacles)):
                id_obs = self.pos_obstacles[i][0]
                list_of_ids.append(id_obs)
                self.add_key(id_obs)
            
            self.mybigdict = self.delete_keys(list_of_ids,self.mybigdict)

            

            

            for i in range(len(self.pos_obstacles)):
                
                id_obs = self.pos_obstacles[i][0]
                x_obs = self.pos_obstacles[i][1]
                y_obs = self.pos_obstacles[i][2]
                
                
                
                
                if id_obs <1000:# pedestrian
                    # obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                    # near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                    # print("###########################################  ", near_id_obs, "  #############################################")
                    # if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>0 and near_id_obs<self.look_ahead_index:
                    #     goal_id = near_id_obs -2 
                    #     flag_cond = True
                    #     print("triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
                    # if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>near_id:
                    #     goal_id = near_id_obs -2 
                    #     flag_cond = True
                    #     print("triggggeeerrrreeedddddd nooooooooooooooooohhhhhhhhhhhhhere")
                    # if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs==0:
                    #     goal_id = np.asarray([self.waypoints.shape[0]-2])
                    #     flag_cond = True
                    #     print("triggggeeerrrreeedddddd thereeeeeee")
                    # if near_id_obs<goal_id and near_id_obs> near_id:
                    #     goal_id = near_id_obs-2
                    #     flag_cond = True
                    #     print("triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                    # if goal_id>=near_id_obs-1 and goal_id<= near_id_obs+1:
                    #     goal_id = near_id_obs -2
                    #     flag_cond = True 
                    #     print("triggggeerrrreeeedddddddddddddddddd behinnnnnnddddddddddddddd")

                    # # print("near id: ", near_id_obs," goal_id ",goal_id)
                    # if (goal_id > self.waypoints.shape[0]-1):
                    #     goal_id = goal_id - self.waypoints.shape[0] 
                    
                    # if ((near_id_obs>=22 and near_id_obs<29) or (near_id_obs>=0 and near_id_obs<=6)) and ((goal_id>=22 and goal_id<29) or (goal_id>=0 and goal_id<=6)) and flag_cond == False:
                    #     print(near_id, type(near_id))
                    #     print(goal_id, type(goal_id))
                    #     if near_id>33:
                    #         targetwaypoints1 = self.waypoints[near_id[0]:]
                    #         targetwaypoints2 = self.waypoints[: goal_id[0]+1]
                    #         targetwaypoints = np.vstack((targetwaypoints1, targetwaypoints2))
                    #     else:    
                    #         targetwaypoints = self.waypoints[near_id[0]:goal_id[0]+1]
                    #     print(targetwaypoints)
                    #     near_id_obs2_, __ = do_kdtree(targetwaypoints[:,:-1], obspos, k =1) 
                    #     # print(near_id_obs2_)
                    #     near_id_obs2, ____= do_kdtree(self.waypoints[:,:-1], targetwaypoints[near_id_obs2_,:-1], k =1)
                    #     print(near_id_obs2)

                    #     if near_id_obs2<goal_id and near_id_obs2> near_id:
                    #         goal_id = near_id_obs2-2
                    #         # flag_cond = True
                    #         print("newwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                    #     if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs2>0 and near_id_obs2<self.look_ahead_index:
                    #         goal_id = near_id_obs2 -2 
                    #         # flag_cond = True
                    #         print("newwwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
                    #     if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs2==0:
                    #         goal_id = np.asarray([self.waypoints.shape[0]-2])
                    #         # flag_cond = True
                    #         print("newwwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd thereeeeeee")
                    #     # if goal_id<0:
                    #     #     goal_id =  self.waypoints.shape[0]+goal_id
                        
                    #     # if goal_id>=near_id_obs2-1 and goal_id<= near_id_obs2+1:
                    #     #     goal_id = near_id_obs2 -2 
                    #     #     print("newwwwwwwwwwwwwwwwww triggggeerrrreeeedddddddddddddddddd behinnnnnnddddddddddddddd")
                    counter = self.mybigdict[id_obs]
                    print("hehuhahahahahahahahhaahah", counter)
                    if counter <20:
                    
                        # coming behind
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
                            goal_id = np.asarray([self.waypoints.shape[0]-2])
                            flag_cond = True
                            print("triggggeeerrrreeedddddd thereeeeeee")
                        if near_id_obs<goal_id and near_id_obs> near_id:
                            goal_id = near_id_obs-2
                            flag_cond = True
                            print("triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                        if goal_id>=near_id_obs-1 and goal_id<= near_id_obs+1:
                            goal_id = near_id_obs -2
                            flag_cond = True 
                            print("triggggeerrrreeeedddddddddddddddddd behinnnnnnddddddddddddddd")

                        # print("near id: ", near_id_obs," goal_id ",goal_id)
                        if (goal_id > self.waypoints.shape[0]-1):
                            goal_id = goal_id - self.waypoints.shape[0] 
                        
                        if ((near_id_obs>=22 and near_id_obs<=26) or (near_id_obs>=0 and near_id_obs<=3)) and ((goal_id>=22 and goal_id<=26) or (goal_id>=0 and goal_id<=3)) and flag_cond == False:
                            print(near_id, type(near_id))
                            print(goal_id, type(goal_id))
                            if near_id>33:
                                targetwaypoints1 = self.waypoints[near_id[0]:]
                                targetwaypoints2 = self.waypoints[: goal_id[0]+1]
                                targetwaypoints = np.vstack((targetwaypoints1, targetwaypoints2))
                            else:    
                                targetwaypoints = self.waypoints[near_id[0]:goal_id[0]+1]
                            print(targetwaypoints)
                            near_id_obs2_, __ = do_kdtree(targetwaypoints[:,:-1], obspos, k =1) 
                            # print(near_id_obs2_)
                            near_id_obs2, ____= do_kdtree(self.waypoints[:,:-1], targetwaypoints[near_id_obs2_,:-1], k =1)
                            print(near_id_obs2)

                            if near_id_obs2<goal_id and near_id_obs2> near_id:
                                goal_id = near_id_obs2-2
                                # flag_cond = True
                                print("newwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                            if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs2>0 and near_id_obs2<self.look_ahead_index:
                                goal_id = near_id_obs2 -2 
                                # flag_cond = True
                                print("newwwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
                            if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs2==0:
                                goal_id = np.asarray([self.waypoints.shape[0]-2])
                                # flag_cond = True
                                print("newwwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd thereeeeeee")
                            # if goal_id<0:
                            #     goal_id =  self.waypoints.shape[0]+goal_id
                            
                            # if goal_id>=near_id_obs2-1 and goal_id<= near_id_obs2+1:
                            #     goal_id = near_id_obs2 -2 
                            #     print("newwwwwwwwwwwwwwwwww triggggeerrrreeeedddddddddddddddddd behinnnnnnddddddddddddddd")

                        else:
                            if (goal_id > self.waypoints.shape[0]-1):
                                goal_id = goal_id - self.waypoints.shape[0]
                            obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                            near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                            print("###########################################  ", near_id_obs, "  #############################################")
                            if goal_id>near_id_obs-1 and goal_id< near_id_obs+1:
                                goal_id = goal_id+3
                                print("triggggeerrrreeeedddddddddddddddddd ahhhhhheeeeaaaadddddddddddddddddd")



                if id_obs ==1015:# tukable obstacle
                    self.countercheck = self.countercheck+1
                    print("hehuhahahahahahahahhaahahhahahahahahahahah", self.countercheck)
                    if self.countercheck >250:
                        self.mybigdict[id_obs]=0

                    counter = self.mybigdict[id_obs]
                    print("hehuhahahahahahahahhaahah", counter)
                    if counter <20:
                    
                        # coming behind
                        obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                        near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                        print("###########################################  ", near_id_obs, "  #############################################")
                        if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>0 and near_id_obs<self.look_ahead_index and goal_id>near_id_obs:
                            goal_id = near_id_obs -2 
                            flag_cond = True
                            print("triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
                            self.mybigdict[id_obs] +=1
                        if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>near_id:
                            goal_id = near_id_obs -2 
                            flag_cond = True
                            print("triggggeeerrrreeedddddd nooooooooooooooooohhhhhhhhhhhhhere")
                            self.mybigdict[id_obs] +=1
                        if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs==0:
                            goal_id = np.asarray([self.waypoints.shape[0]-2])
                            flag_cond = True
                            print("triggggeeerrrreeedddddd thereeeeeee")
                            self.mybigdict[id_obs] +=1
                        if near_id_obs<goal_id and near_id_obs> near_id:
                            goal_id = near_id_obs-2
                            flag_cond = True
                            print("triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                            self.mybigdict[id_obs] +=1
                        if goal_id>=near_id_obs-1 and goal_id<= near_id_obs+1:
                            goal_id = near_id_obs -2
                            flag_cond = True 
                            print("triggggeerrrreeeedddddddddddddddddd behinnnnnnddddddddddddddd")
                            self.mybigdict[id_obs] +=1

                        # print("near id: ", near_id_obs," goal_id ",goal_id)
                        if (goal_id > self.waypoints.shape[0]-1):
                            goal_id = goal_id - self.waypoints.shape[0] 
                        
                        if ((near_id_obs>=22 and near_id_obs<=26) or (near_id_obs>=0 and near_id_obs<=3)) and ((goal_id>=22 and goal_id<=26) or (goal_id>=0 and goal_id<=3)) and flag_cond == False:
                            print(near_id, type(near_id))
                            print(goal_id, type(goal_id))
                            if near_id>33:
                                targetwaypoints1 = self.waypoints[near_id[0]:]
                                targetwaypoints2 = self.waypoints[: goal_id[0]+1]
                                targetwaypoints = np.vstack((targetwaypoints1, targetwaypoints2))
                            else:    
                                targetwaypoints = self.waypoints[near_id[0]:goal_id[0]+1]
                            print(targetwaypoints)
                            near_id_obs2_, __ = do_kdtree(targetwaypoints[:,:-1], obspos, k =1) 
                            # print(near_id_obs2_)
                            near_id_obs2, ____= do_kdtree(self.waypoints[:,:-1], targetwaypoints[near_id_obs2_,:-1], k =1)
                            print(near_id_obs2)

                            if near_id_obs2<goal_id and near_id_obs2> near_id:
                                goal_id = near_id_obs2-2
                                # flag_cond = True
                                print("newwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                                self.mybigdict[id_obs] +=1
                            if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs2>0 and near_id_obs2<self.look_ahead_index:
                                goal_id = near_id_obs2 -2 
                                # flag_cond = True
                                print("newwwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
                                self.mybigdict[id_obs] +=1
                            if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs2==0:
                                goal_id = np.asarray([self.waypoints.shape[0]-2])
                                # flag_cond = True
                                print("newwwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd thereeeeeee")
                                self.mybigdict[id_obs] +=1
                            # if goal_id<0:
                            #     goal_id =  self.waypoints.shape[0]+goal_id
                            
                            # if goal_id>=near_id_obs2-1 and goal_id<= near_id_obs2+1:
                            #     goal_id = near_id_obs2 -2 
                            #     print("newwwwwwwwwwwwwwwwww triggggeerrrreeeedddddddddddddddddd behinnnnnnddddddddddddddd")
                        
                        self.countercheck = 0

                    else:
                        if (goal_id > self.waypoints.shape[0]-1):
                            goal_id = goal_id - self.waypoints.shape[0]
                        obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                        near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                        print("###########################################  ", near_id_obs, "  #############################################")
                        if goal_id>near_id_obs-1 and goal_id< near_id_obs+1:
                            goal_id = goal_id+3
                            print("triggggeerrrreeeedddddddddddddddddd ahhhhhheeeeaaaadddddddddddddddddd")
                    
                    

                if id_obs ==1017: #dynamic obstacle
                    print("i am in 1017")

                    self.countercheck = self.countercheck+1
                    print("hehuhahahahahahahahhaahahhahahahahahahahah", self.countercheck)
                    if self.countercheck >250:
                        self.mybigdict[id_obs]=0

                    counter = self.mybigdict[id_obs]
                    print("hehuhahahahahahahahhaahah", counter)
                    if counter <20:
                    
                        # coming behind
                        obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                        near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                        print("###########################################  ", near_id_obs, "  #############################################")
                        if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>0 and near_id_obs<self.look_ahead_index and goal_id>near_id_obs:
                            goal_id = near_id_obs -2 
                            flag_cond = True
                            print("triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
                            self.mybigdict[id_obs] +=1
                        if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>near_id:
                            goal_id = near_id_obs -2 
                            flag_cond = True
                            print("triggggeeerrrreeedddddd nooooooooooooooooohhhhhhhhhhhhhere")
                            self.mybigdict[id_obs] +=1
                        if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs==0:
                            goal_id = np.asarray([self.waypoints.shape[0]-2])
                            flag_cond = True
                            print("triggggeeerrrreeedddddd thereeeeeee")
                            self.mybigdict[id_obs] +=1
                        if near_id_obs<goal_id and near_id_obs> near_id:
                            goal_id = near_id_obs-2
                            flag_cond = True
                            print("triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                            self.mybigdict[id_obs] +=1
                        if goal_id>=near_id_obs-1 and goal_id<= near_id_obs+1:
                            goal_id = near_id_obs -2
                            flag_cond = True 
                            print("triggggeerrrreeeedddddddddddddddddd behinnnnnnddddddddddddddd")
                            self.mybigdict[id_obs] +=1

                        # print("near id: ", near_id_obs," goal_id ",goal_id)
                        if (goal_id > self.waypoints.shape[0]-1):
                            goal_id = goal_id - self.waypoints.shape[0] 
                        
                        if ((near_id_obs>=22 and near_id_obs<=26) or (near_id_obs>=0 and near_id_obs<=3)) and ((goal_id>=22 and goal_id<=26) or (goal_id>=0 and goal_id<=3)) and flag_cond == False:
                            print(near_id, type(near_id))
                            print(goal_id, type(goal_id))
                            if near_id>33:
                                targetwaypoints1 = self.waypoints[near_id[0]:]
                                targetwaypoints2 = self.waypoints[: goal_id[0]+1]
                                targetwaypoints = np.vstack((targetwaypoints1, targetwaypoints2))
                            else:    
                                targetwaypoints = self.waypoints[near_id[0]:goal_id[0]+1]
                            print(targetwaypoints)
                            near_id_obs2_, __ = do_kdtree(targetwaypoints[:,:-1], obspos, k =1) 
                            # print(near_id_obs2_)
                            near_id_obs2, ____= do_kdtree(self.waypoints[:,:-1], targetwaypoints[near_id_obs2_,:-1], k =1)
                            print(near_id_obs2)

                            if near_id_obs2<goal_id and near_id_obs2> near_id:
                                goal_id = near_id_obs2-2
                                # flag_cond = True
                                print("newwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                                self.mybigdict[id_obs] +=1
                            if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs2>0 and near_id_obs2<self.look_ahead_index:
                                goal_id = near_id_obs2 -2 
                                # flag_cond = True
                                print("newwwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
                                self.mybigdict[id_obs] +=1
                            if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs2==0:
                                goal_id = np.asarray([self.waypoints.shape[0]-2])
                                # flag_cond = True
                                print("newwwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd thereeeeeee")
                                self.mybigdict[id_obs] +=1
                            # if goal_id<0:
                            #     goal_id =  self.waypoints.shape[0]+goal_id
                            
                            # if goal_id>=near_id_obs2-1 and goal_id<= near_id_obs2+1:
                            #     goal_id = near_id_obs2 -2 
                            #     print("newwwwwwwwwwwwwwwwww triggggeerrrreeeedddddddddddddddddd behinnnnnnddddddddddddddd")
                        
                        self.countercheck = 0

                    else:
                        if (goal_id > self.waypoints.shape[0]-1):
                            goal_id = goal_id - self.waypoints.shape[0]
                        obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                        near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                        print("###########################################  ", near_id_obs, "  #############################################")
                        if goal_id>near_id_obs-1 and goal_id< near_id_obs+1:
                            goal_id = goal_id+3
                            print("triggggeerrrreeeedddddddddddddddddd ahhhhhheeeeaaaadddddddddddddddddd")
                    # self.countercheck = self.countercheck+1
                    # print("hehuhahahahahahahahhaahahhahahahahahahahah", self.countercheck)
                    # if self.countercheck >250:
                    #     self.mybigdict[id_obs]=0

                    # counter = self.mybigdict[id_obs]
                    # print("hehuhahahahahahahahhaahah", counter)
                    
                    # variablecheck = 1
                    # if counter <20:
                    
                    #     # coming behind
                    #     obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                    #     near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                    #     print("###########################################  ", near_id_obs, "  #############################################")
                    #     if near_id_obs<6 and near_id_obs>=0:
                    #         variablecheck =2
                    #     if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>0 and near_id_obs<self.look_ahead_index and goal_id>near_id_obs:
                    #         goal_id = near_id_obs -variablecheck 
                    #         flag_cond = True
                    #         print("triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
                    #         self.mybigdict[id_obs] +=1
                    #     if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs>near_id:
                    #         goal_id = near_id_obs -variablecheck 
                    #         flag_cond = True
                    #         print("triggggeeerrrreeedddddd nooooooooooooooooohhhhhhhhhhhhhere")
                    #         self.mybigdict[id_obs] +=1
                    #     if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs==0:
                    #         goal_id = np.asarray([self.waypoints.shape[0]-variablecheck])
                    #         flag_cond = True
                    #         print("triggggeeerrrreeedddddd thereeeeeee")
                    #         self.mybigdict[id_obs] +=1
                    #     if near_id_obs<goal_id and near_id_obs> near_id:
                    #         goal_id = near_id_obs-variablecheck
                    #         flag_cond = True
                    #         print("triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                    #         self.mybigdict[id_obs] +=1
                    #     if goal_id>=near_id_obs-1 and goal_id<= near_id_obs+1:
                    #         goal_id = near_id_obs -variablecheck
                    #         flag_cond = True 
                    #         print("triggggeerrrreeeedddddddddddddddddd behinnnnnnddddddddddddddd")
                    #         self.mybigdict[id_obs] +=1

                    #     # print("near id: ", near_id_obs," goal_id ",goal_id)
                    #     if (goal_id > self.waypoints.shape[0]-1):
                    #         goal_id = goal_id - self.waypoints.shape[0] 
                        
                    #     if ((near_id_obs>=22 and near_id_obs<=26) or (near_id_obs>=0 and near_id_obs<=3)) and ((goal_id>=22 and goal_id<=26) or (goal_id>=0 and goal_id<=3)) and flag_cond == False:
                    #         print(near_id, type(near_id))
                    #         print(goal_id, type(goal_id))
                    #         if near_id>33:
                    #             targetwaypoints1 = self.waypoints[near_id[0]:]
                    #             targetwaypoints2 = self.waypoints[: goal_id[0]+1]
                    #             targetwaypoints = np.vstack((targetwaypoints1, targetwaypoints2))
                    #         else:    
                    #             targetwaypoints = self.waypoints[near_id[0]:goal_id[0]+1]
                    #         print(targetwaypoints)
                    #         near_id_obs2_, __ = do_kdtree(targetwaypoints[:,:-1], obspos, k =1) 
                    #         # print(near_id_obs2_)
                    #         near_id_obs2, ____= do_kdtree(self.waypoints[:,:-1], targetwaypoints[near_id_obs2_,:-1], k =1)
                    #         print(near_id_obs2)

                    #         if near_id_obs2<goal_id and near_id_obs2> near_id:
                    #             goal_id = near_id_obs2-variablecheck
                    #             # flag_cond = True
                    #             print("newwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd wwwwwwwwwwwwhere")
                    #             self.mybigdict[id_obs] +=1
                    #         if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs2>0 and near_id_obs2<self.look_ahead_index:
                    #             goal_id = near_id_obs2 -variablecheck 
                    #             # flag_cond = True
                    #             print("newwwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd hhhhhhhhhhhhhere")
                    #             self.mybigdict[id_obs] +=1
                    #         if near_id> self.waypoints.shape[0]-1-self.look_ahead_index and near_id_obs2==0:
                    #             goal_id = np.asarray([self.waypoints.shape[0]-variablecheck])
                    #             # flag_cond = True
                    #             print("newwwwwwwwwwwwwwwwwww triggggeeerrrreeedddddd thereeeeeee")
                    #             self.mybigdict[id_obs] +=1
                    #         # if goal_id<0:
                    #         #     goal_id =  self.waypoints.shape[0]+goal_id
                            
                    #         # if goal_id>=near_id_obs2-1 and goal_id<= near_id_obs2+1:
                    #         #     goal_id = near_id_obs2 -2 
                    #         #     print("newwwwwwwwwwwwwwwwww triggggeerrrreeeedddddddddddddddddd behinnnnnnddddddddddddddd")
                        
                    #     self.countercheck = 0

                    # else:
                    #     if (goal_id > self.waypoints.shape[0]-1):
                    #         goal_id = goal_id - self.waypoints.shape[0]
                    #     obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                    #     near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                    #     print("###########################################  ", near_id_obs, "  #############################################")
                    #     if goal_id>near_id_obs-1 and goal_id< near_id_obs+1:
                    #         goal_id = goal_id+3
                    #         print("triggggeerrrreeeedddddddddddddddddd ahhhhhheeeeaaaadddddddddddddddddd")







                #     obspos = np.asarray([x_obs, y_obs]).reshape(1,-1)
                #     near_id_obs, __ = do_kdtree(self.waypoints[:,:-1], obspos, k =1)
                #     print("###########################################  ", near_id_obs, "  #############################################")
                #     if goal_id>near_id_obs-1 and goal_id< near_id_obs+1:
                #         goal_id = goal_id+3
                #         print("triggggeerrrreeeedddddddddddddddddd ahhhhhheeeeaaaadddddddddddddddddd")


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
        print("before: ",self.mybigdict)
           

        if len(self.pos_obstacles)==0:
            self.mybigdict = self.delete_keys(list_of_ids,self.mybigdict)

        print("after: ",self.mybigdict)
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
    
    def add_key(self,key):
        if key not in self.mybigdict:   
            # if self.mybigdict[key]<5:
            # self.mybigdict[key]+=1
            self.mybigdict[key]=0
            # else:
            #     self.mybigdict[key]=0
        # else:
        #     self.mybigdict[key]=0

    def delete_keys(self,input_list,input_dict):

        if len(input_list)!=0:

            if all(elem in input_dict for elem in input_list):
                return input_dict
            
            new_dict = {key:input_dict[key] for key in input_list if key in input_dict}
            return new_dict

        else:
            print("pls delete kar bhai")
            new_dict ={}
            return new_dict
        
    
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
            if (car_id ==1015 and pos_v <=0.05):
                pos_obs.append([car_id, pos_x, pos_y])
            if (car_id ==1017 and pos_v <=0.05):
                pos_obs.append([car_id, pos_x, pos_y])
            i=i+5
        
        return pos_obs
    
    def obstacle_callback(self,msg):
        self.pos_obstacles = self.get_obstacle_pose(msg, self.scale_factor, self.translate_x, self.translate_y )
    
    def perception_loss_callback(self,msg):
        print("annnndddhhhhaaaaaa aannndddaaaaaaa")
        lost_car_pos = self.get_obstacle_pose(msg, self.scale_factor, self.translate_x, self.translate_y )
        for i in range(len(lost_car_pos)):
            self.pos_obstacles.append(lost_car_pos[i])

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