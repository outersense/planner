import numpy as np
import os
import rosbag
import matplotlib.pyplot as plt
from math import pi as PI
import sys
import tf
import scipy.spatial as sp
from sklearn.neighbors import NearestNeighbors
import cv2
from nav_msgs.msg import Odometry
import math





bagfile_path = "/home/dhanesh/Masters/OuterSense/Planning/rosbags/oct17"
bagfile_name = "1_car_1_loop.bag"

topic_gps = "/rccar_pose"
topic_car2_world = "/car2/fused"


bagfile_ego = os.path.join(bagfile_path, bagfile_name)
bag_ego = rosbag.Bag(bagfile_ego)

##################################################################################
def get_heading(pose):
    '''Converts orientation from quarternions to euler angles
    Args: 
        1) pose: contains position xyz and orientation as a quarternion
    Returns:
        1) heading: heading in euler angles
    '''
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    if yaw < 0:
        # print("yaw angle is negative and equals", yaw )
        yaw += 2*PI 
    if roll < 0:
        # print("yaw angle is negative and equals", yaw )
        roll += 2*PI 
    if pitch < 0:
        # print("yaw angle is negative and equals", yaw )
        pitch += 2*PI         
    yaw_degree = yaw*180/PI
    roll_degree = roll*180/PI
    pitch_degree = pitch*180/PI
    # if yaw_degree < 0:

    return [roll_degree, pitch_degree, yaw_degree], yaw_degree


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


def calculate_perpendicular_points(waypoints):
    x, y, theta = waypoints[:, 0], waypoints[:, 1], waypoints[:, 2]

    # Calculate offsets for x and y based on the given theta
    x_offset = 0.35 * np.cos(theta + np.pi / 2)
    y_offset = 0.35 * np.sin(theta + np.pi / 2)

    # Calculate the coordinates for the points on one side of the original point
    x1 = x + x_offset
    y1 = y + y_offset

    # Calculate the coordinates for the points on the other side of the original point
    x2 = x - x_offset
    y2 = y - y_offset

    # Stack the x and y coordinates into two separate arrays
    perpendicular_points_1 = np.column_stack((x1, y1))
    perpendicular_points_2 = np.column_stack((x2, y2))

    return perpendicular_points_1, perpendicular_points_2

def parse_odom_message(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    # Extracting the orientation information (quaternion to euler)
    # list_angles, theta = get_heading(msg.pose.pose)
    x_orient = msg.pose.pose.orientation.x
    y_orient = msg.pose.pose.orientation.y
    z_orient = msg.pose.pose.orientation.z
    w_orient = msg.pose.pose.orientation.w
    theta = math.atan2(2 * (w_orient * z_orient + x_orient * y_orient), 1 - 2 * (y_orient**2 + z_orient**2))

    # Extracting the linear velocity
    linear_x = msg.twist.twist.linear.x
    linear_y = msg.twist.twist.linear.y
    linear_z = msg.twist.twist.linear.z
    velocity = np.sqrt(linear_x**2 + linear_y**2 + linear_z**2)

    return x, y, z, theta, velocity, [x, y,theta]

# Initialising and errors
gps_msg_count = bag_ego.get_message_count(topic_filters= topic_gps)
gps_pose_array = np.zeros(shape=(gps_msg_count,5))
if gps_msg_count ==0:
    print("no gps dynamic global pose availabe")

gps_msg_count2 = bag_ego.get_message_count(topic_filters= topic_car2_world)
gps_pose_array2 = np.zeros(shape=(gps_msg_count2,3))
if gps_msg_count2 ==0:
    print("no gps dynamic global pose 2 availabe")


gps_msgs = 0
gps_msgs2 = 0
for topic, msg, bag_t in bag_ego.read_messages():
    if topic == topic_gps:
        heading_degree_gps, location_gps, pose_gps = get_rccar_pose(msg)
        # print(location_gps)
        gps_pose_array[gps_msgs] = pose_gps
        gps_msgs += 1

    if topic == topic_car2_world:
        x, y, z, theta, vel, pose_odom = parse_odom_message(msg)
        # print(location_gps)
        gps_pose_array2[gps_msgs2] = pose_odom
        gps_msgs2 += 1
yaw_array = gps_pose_array2[:,-1]#*180/PI
x_points = gps_pose_array2[::5,0]
y_points = gps_pose_array2[::5,1]
yaw_points = yaw_array[::5]
waypoints = np.stack((x_points,y_points, yaw_points)).T
waypoints = waypoints[5:,:]

np.save("waypoints.npy", waypoints)

t_cos = np.cos(waypoints[:,2])
t_sin = np.sin(waypoints[:,2])

# print(x_points, y_points)
print(waypoints, waypoints.shape)

# print(gps_pose_array, gps_pose_array.shape)
# plt.plot(x_points[-1], y_points[-1], 'r+', label="Lidar after extrinsics")
plt.plot(waypoints[:,0], waypoints[:,1], 'b+', label="Lidar after extrinsics")
plt.quiver(waypoints[:,0], waypoints[:,1], t_cos, t_sin, color='lightblue', width = 0.005, angles = 'uv' )
plt.show()
plt.close()

# plt.plot(gps_pose_array2[:,1], gps_pose_array2[:,2], 'r+', label="Lidar after extrinsics")
# plt.show()
# plt.close()

