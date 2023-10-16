import numpy as np
import os
import rosbag
import matplotlib.pyplot as plt
from math import pi as PI
import sys
import scipy.spatial as sp
from sklearn.neighbors import NearestNeighbors
from copy import deepcopy




bagfile_path = "/home/dhanesh/Masters/OuterSense/Planning/rosbags/oct9"
bagfile_name = "1.bag"

topic_gps = "/rccar_pose"


bagfile_ego = os.path.join(bagfile_path, bagfile_name)
bag_ego = rosbag.Bag(bagfile_ego)

filename_gpscsv = "data_writer_mission_planner.csv"

##################################################################################

def get_rccar_pose(msg):
    '''Takes in a GPS message and returns the GPS pose
    Args: 
        1) msg: dynamicglobalpose, dynamicglobalposeim, dynamicglobalposeecho, DynamicGlobalPosewithCovar msg type
    Returns:
        1) heading: heading in euler angles
        2) location: [x,y, z]
        3) pose: list [x,y, z,heading] 
    '''
    # msg_ts_gps = msg.header.stamp.to_sec()
    pose = msg.data
    # pose_rccar = np.asarray(msg.data)
    car_id = pose[0]
    pos_x = pose[1]
    pos_y = pose[2]
    pos_v = pose[3]
    pos_yaw = pose[4]
    location = [pos_x, pos_y]
    heading_degree = pos_yaw*180/PI
    # pose.append(msg_ts_gps)
    return heading_degree, location, pose[:5]

# Initialising and errors
gps_msg_count = bag_ego.get_message_count(topic_filters= topic_gps)
gps_pose_array = np.zeros(shape=(gps_msg_count,5))
if gps_msg_count ==0:
    print("no gps dynamic global pose availabe")


gps_msgs = 0
for topic, msg, bag_t in bag_ego.read_messages():
    if topic == topic_gps:
        heading_degree_gps, location_gps, pose_gps = get_rccar_pose(msg)
        # print(location_gps)
        gps_pose_array[gps_msgs] = pose_gps
        gps_msgs += 1

print(gps_pose_array, gps_pose_array.shape)
plt.plot(gps_pose_array[:,1], gps_pose_array[:,2], 'b+', label="Lidar after extrinsics")
plt.show()
plt.close()













