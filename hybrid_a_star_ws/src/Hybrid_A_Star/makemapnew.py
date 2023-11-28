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
from skimage.transform import rotate




bagfile_path = "/home/dhanesh/Masters/OuterSense/Planning_test/planner/rosbags/nov10"
# bagfile_path = "/home/dhanesh/Masters/OuterSense/Planning_new/planner/rosbags/nov2"
# bagfile_path = "/home/dhanesh/Masters/OuterSense/Planning_new/planner/rosbags/nov1"
bagfile_name = "manual_jash2.bag"
# bagfile_name = "manual_ronit.bag"

topic_gps = "/rccar_pose"
topic_car2_world = "/car2/world_pose"

scale_factor =100
# scale_factor =10


bagfile_ego = os.path.join(bagfile_path, bagfile_name)
bag_ego = rosbag.Bag(bagfile_ego)

filename_gpscsv = "data_writer_mission_planner.csv"

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

# def get_carworld_pose(msg):
#     '''Takes in a GPS message and returns the GPS pose
#     Args: 
#         1) msg: dynamicglobalpose, dynamicglobalposeim, dynamicglobalposeecho, DynamicGlobalPosewithCovar msg type
#     Returns:
#         1) heading: heading in euler angles
#         2) location: [x,y, z]
#         3) pose: list [x,y, z,heading] 
#     '''
#     # msg_ts_gps = msg.header.stamp.to_sec()
#     pose = msg.pose
    
#     # pose_rccar = np.asarray(msg.data)
#     # car_id = pose[0]
#     pos_x = pose.position.x
#     pos_y = pose.position.y
#     # pos_v = pose[3]
#     pos_rpy, pos_yaw = get_heading(pose)
#     location = [pos_x, pos_y]
#     # heading_degree = pos_yaw*180/PI
#     # pose.append(msg_ts_gps)
#     pose_ =[pos_x, pos_y, pos_yaw]
#     return pos_yaw, location, pose_
def calculate_perpendicular_points(waypoints):
    x, y, theta = waypoints[:, 0], waypoints[:, 1], waypoints[:, 2]
    perpendicular_points_1 = np.zeros((waypoints.shape[0],2))
    perpendicular_points_2 = np.zeros((waypoints.shape[0],2))
    # Calculate offsets for x and y based on the given theta
    for i in range(waypoints.shape[0]):
        x = waypoints[i,0]
        y = waypoints[i,1]
        theta = waypoints[i,2]
    
    
        if (0<= theta + PI / 2< 1.57):
            x_offset = 0.45 * np.cos(theta + PI / 2)
            y_offset = 0.45 * np.sin(theta + PI / 2)
        if (1.57<= theta + PI / 2< 3.14):
            x_offset = 0.45 * np.cos(theta + PI / 2)
            y_offset = 0.45 * np.sin(theta + PI / 2)
        if (3.14<= theta + PI / 2< 4.71):
            x_offset = 0.45 * np.cos(theta + PI / 2)
            y_offset = 0.45 * np.sin(theta + PI / 2)
        if (4.71<= theta + PI / 2< 6.28):
            x_offset = 0.45 * np.cos(theta + PI / 2)
            y_offset = 0.45 * np.sin(theta + PI / 2)

    # Calculate the coordinates for the points on one side of the original point
    
        perpendicular_points_1[i,0] = x + x_offset
        perpendicular_points_1[i,1] = y + y_offset
        # perpendicular_points_1[i,0] = 
        # Calculate the coordinates for the points on the other side of the original point
        perpendicular_points_2[i,0] = x - x_offset
        perpendicular_points_2[i,1] = y - y_offset
    pp1 = perpendicular_points_2[:12]
    pp2 = perpendicular_points_1[12:30]
    pp3 = perpendicular_points_2[30:]

    pp2_ = np.vstack((pp1,pp2,pp3))
    pp1 = perpendicular_points_1[:12]
    pp2 = perpendicular_points_2[12:30]
    pp3 = perpendicular_points_1[30:]

    pp1_ = np.vstack((pp1,pp2,pp3))


    # Stack the x and y coordinates into two separate arrays
    # perpendicular_points_1 = np.column_stack((x1, y1))
    # perpendicular_points_2 = np.column_stack((x2, y2))

    return pp1_, pp2_
    # return perpendicular_points_1, perpendicular_points_2

def calculate_perpendicular_points2(waypoints):
    lane_1 = []
    lane_2 = []

    for waypoint in waypoints:
        x, y, theta = waypoint

        perpendicular_heading = theta + np.pi / 2  # Rotate the heading by 90 degrees to get a perpendicular direction
        x_offset = 0.35 * np.cos(perpendicular_heading)
        y_offset = 0.35 * np.sin(perpendicular_heading)

        lane_1_point = [x + x_offset, y + y_offset]
        lane_2_point = [x - x_offset, y - y_offset]

        lane_1.append(lane_1_point)
        lane_2.append(lane_2_point)

    return np.array(lane_1), np.array(lane_2)

def create_road_image(pp1, pp2):
    # Find the minimum and maximum coordinates from both pp1 and pp2
    min_x = min(np.min(pp1[:, 0]), np.min(pp2[:, 0]))
    max_x = max(np.max(pp1[:, 0]), np.max(pp2[:, 0]))
    min_y = min(np.min(pp1[:, 1]), np.min(pp2[:, 1]))
    max_y = max(np.max(pp1[:, 1]), np.max(pp2[:, 1]))



    # Set the image size to accommodate the extreme points
    # image_size = (int(max_x - min_x) + 703, int(max_y - min_y) + 500)
    print(min_x, min_y, max_x, max_y)
    image_size = (int(max_x - min_x), int(max_y - min_y))
    print("size is ", image_size)

    # Create a black image
    road_image = np.zeros((image_size[1], image_size[0], 3), dtype=np.uint8)

    # Shift the points to fit within the image size
    pp1_shifted = pp1 - [min_x, min_y]
    pp2_shifted = pp2 - [min_x, min_y]

    # Rescale the shifted points to the image size
    pp1_rescaled = pp1_shifted.astype(int) #(pp1_shifted * (image_size[0] / (max_x - min_x))).astype(int)
    pp2_rescaled = pp2_shifted.astype(int) #(pp2_shifted * (image_size[0] / (max_x - min_x))).astype(int)

    print(pp1_rescaled, pp2_rescaled)

    # Draw the road boundaries as white lines
    for i in range(len(pp1_rescaled) - 1):
        cv2.line(road_image, tuple(pp1_rescaled[i]), tuple(pp2_rescaled[i]), (255, 0, 0), 5)

    # Fill the road area with white color
    cv2.fillPoly(road_image, [pp1_rescaled, pp2_rescaled[::-1]], (255, 255, 255))
    points = np.asarray([[4.4969 +0.2,                0.7840],
                         [3.3157 +0.2,       4.4421e-01 -0.1],
                         [2.6240 -0.2,       7.8378e-01 +0.05],
                         [3.4319,           1.1545]])
    points_shifted = points*100 - [min_x, min_y]

    # # Rescale the shifted points to the image size
    # points_rescaled = (points_shifted * (image_size[0] / (max_x - min_x))).astype(int)
    # print(points_rescaled, points_rescaled.shape)

    # points_rescaled = np.asarray([[430+10+20, 129   +8],
    #                               [309+15+20,  95-15+8],
    #                               [238-25+20, 129+15+8],
    #                               [321   +20, 168+8 +8]])
    
    # cv2.fillPoly(road_image, [points_shifted.astype(int)], (255, 255, 255))

    return road_image

def create_road_image2(pp1, pp2):
    # Find the minimum and maximum coordinates from both pp1 and pp2
    min_x = min(np.min(pp1[:, 0]), np.min(pp2[:, 0]))
    max_x = max(np.max(pp1[:, 0]), np.max(pp2[:, 0]))
    min_y = min(np.min(pp1[:, 1]), np.min(pp2[:, 1]))
    max_y = max(np.max(pp1[:, 1]), np.max(pp2[:, 1]))



    # Set the image size to accommodate the extreme points
    # image_size = (int(max_x - min_x) + 703, int(max_y - min_y) + 500)
    print(min_x, min_y, max_x, max_y)
    image_size = (int(max_x - min_x), int(max_y - min_y))
    print("size is ", image_size)

    # Create a black image
    road_image = np.ones((image_size[1], image_size[0], 3), dtype=np.uint8)*255
  

    return road_image

def resize_image_to_tenth(image):
    # Get the original image dimensions
    height, width = image.shape[:2]

    # Calculate the new dimensions (1/10th the size)
    new_height = height // 10
    new_width = width // 10

    # Resize the image to the new dimensions
    resized_image = cv2.resize(image, (new_width, new_height))

    return resized_image

def fill_lanes_with_color(lane_1, lane_2):
    # Finding minimum and maximum x, y coordinates for the lanes
    min_x = min(np.min(lane_1[:, 0]), np.min(lane_2[:, 0]))
    max_x = max(np.max(lane_1[:, 0]), np.max(lane_2[:, 0]))
    min_y = min(np.min(lane_1[:, 1]), np.min(lane_2[:, 1]))
    max_y = max(np.max(lane_1[:, 1]), np.max(lane_2[:, 1]))

    # Calculate image size based on the lane coordinates
    print("translate by :            {}             {}".format(-min_x, -min_y))
    image_size = (int(max_x - min_x), int(max_y - min_y))  # Adjust for border if needed

    # Create a black canvas
    image = np.zeros((image_size[1], image_size[0], 3), dtype=np.uint8)

    # Translation values for the lanes to fit them into the image
    translate_x = -min_x
    translate_y = -min_y

    # Translate the lanes to fit in the image
    lane_1[:, 0] += translate_x
    lane_1[:, 1] += translate_y
    lane_2[:, 0] += translate_x
    lane_2[:, 1] += translate_y
    # 5 and 2.1
    # lane_2_indexes= np.where(lane_2[:,0]>(5*scale_factor+translate_x))
    # lane_1_indexes= np.where(lane_1[:,0]<(2.2*scale_factor+translate_x))
    lane_2_indexes= np.where(lane_2[:,0]>(4.7*scale_factor+translate_x))
    lane_1_indexes= np.where(lane_1[:,0]<(2.5*scale_factor+translate_x))
    # print(lane_1_indexes, type(lane_1_indexes))
    lane_3 = lane_1[lane_1_indexes[0]]
    lane_4 = lane_2[lane_2_indexes[0]]

    # Create masks for each lane separately
    mask_lane1 = np.zeros((image_size[1], image_size[0]), dtype=np.uint8)
    mask_lane2 = np.zeros((image_size[1], image_size[0]), dtype=np.uint8)
    mask_lane3 = np.ones((image_size[1], image_size[0]), dtype=np.uint8)*255

    # Draw each lane on its own mask
    cv2.fillPoly(mask_lane1, [lane_1.astype(int)], color=255)
    cv2.fillPoly(mask_lane2, [lane_2.astype(int)], color=255)

    # Combine the lane masks using bitwise OR operation
    combined_mask = cv2.bitwise_or(mask_lane1, mask_lane2)
    cv2.fillPoly(combined_mask, [lane_3.astype(int)], color=0)
    cv2.fillPoly(combined_mask, [lane_4.astype(int)], color=0)

    # Apply the combined mask to the image
    image[combined_mask == 255] = [255, 255, 255]  # Filling the lanes with white

    return image, mask_lane3


# Initialising and errors
gps_msg_count = bag_ego.get_message_count(topic_filters= topic_gps)
gps_pose_array = np.zeros(shape=(gps_msg_count,5))
if gps_msg_count ==0:
    print("no gps dynamic global pose availabe")

# gps_msg_count2 = bag_ego.get_message_count(topic_filters= topic_car2_world)
# gps_pose_array2 = np.zeros(shape=(gps_msg_count2,3))
# if gps_msg_count2 ==0:
#     print("no gps dynamic global pose 2 availabe")


gps_msgs = 0
gps_msgs2 = 0
for topic, msg, bag_t in bag_ego.read_messages():
    if topic == topic_gps:
        heading_degree_gps, location_gps, pose_gps = get_rccar_pose(msg)
        # print(location_gps)
        gps_pose_array[gps_msgs] = pose_gps
        gps_msgs += 1

    # if topic == topic_car2_world:
    #     heading_degree_gps2, location_gps2, pose_gps2 = get_carworld_pose(msg)
    #     # print(location_gps)
    #     gps_pose_array2[gps_msgs2] = pose_gps2
    #     gps_msgs2 += 1
yaw_array = gps_pose_array[:,-1]#*180/PI
x_points = gps_pose_array[::20,1]
y_points = gps_pose_array[::20,2]
yaw_points = yaw_array[::20]
waypoints = np.stack((x_points,y_points, yaw_points)).T[1:,:]
# print(waypoints)
# waypoints = np.asarray([[ 2.51426935e+00,  4.91118938e-01,  3.26359332e-01],
#                         [ 2.79333186e+00,  5.75201929e-01,  2.57981598e-01],
#                         [ 3.09102345e+00,  6.56028569e-01,  2.58095711e-01],
#                         [ 3.39558887e+00,  7.39092469e-01,  2.81085581e-01],
#                         [ 3.68272233e+00,  8.13478172e-01,  2.22499445e-01],
#                         [ 3.98836303e+00,  8.72113407e-01,  1.97800457e-01],
#                         [ 4.41188955e+00,  9.73643243e-01,  2.37397671e-01],
#                         [ 4.71176386e+00,  1.05769050e+00,  2.95446515e-01],
#                         [ 4.99233818e+00,  1.15032542e+00,  3.34062874e-01],
#                         [ 5.28172970e+00,  1.25759184e+00,  3.68352532e-01],
#                         [ 5.25145149e+00,  1.27332699e+00,  4.30903018e-01],
#                         [ 5.51130247e+00,  1.40376687e+00,  3.80270809e-01],
#                         [ 5.80086231e+00,  1.50306058e+00,  2.90026993e-01],
#                         [ 6.08850861e+00,  1.58499253e+00,  2.02817425e-01],
#                         [ 6.35881805e+00,  1.60528743e+00, -3.68811237e-03],
#                         [ 6.54190445e+00,  1.58833456e+00, -2.54994035e-01],
#                         [ 6.86892843e+00,  1.54701471e+00, -4.49753076e-01],
#                         [ 7.05560541e+00,  1.41516900e+00, -7.83347309e-01],
#                         [ 7.18132019e+00,  1.25668979e+00, -1.05358374e+00],
#                         [ 7.26306677e+00,  1.06298971e+00, -1.27376115e+00],
#                         [ 7.28938007e+00,  8.71293843e-01, -1.57448483e+00],
#                         [ 7.27830219e+00,  6.88031733e-01, -1.73375618e+00],
#                         [ 7.23011112e+00,  5.39327383e-01, -1.99686849e+00],
#                         [ 7.11141348e+00,  3.55717152e-01, -2.23984694e+00],
#                         [ 6.95376635e+00,  2.04741284e-01, -2.53109193e+00],
#                         [ 6.70200729e+00,  8.35612565e-02, -2.84884167e+00],
#                         [ 6.44648361e+00,  5.31803928e-02,  3.13790464e+00],
#                         [ 6.13302183e+00,  6.67227134e-02,  3.05130482e+00],
#                         [ 5.79877710e+00,  1.05757698e-01,  2.99704528e+00],
#                         [ 5.43114853e+00,  1.58164844e-01,  3.02229691e+00],
#                         [ 5.09357262e+00,  1.98849693e-01,  3.02515864e+00],
#                         [ 4.98655319e+00,  1.13634902e-01,  2.96985984e+00],
#                         [ 4.65974760e+00,  1.82978201e-01,  2.84633589e+00],
#                         [ 4.36351156e+00,  2.85982150e-01,  2.76458859e+00],
#                         [ 4.03865957e+00,  4.05983853e-01,  2.83958364e+00],
#                         [ 3.58878255e+00,  5.33763635e-01,  2.81264472e+00],
#                         [ 3.22477007e+00,  6.64591479e-01,  2.79871893e+00],
#                         [ 2.87420344e+00,  0.87368112e+00,  2.78426099e+00],
#                         [ 2.56073594e+00,  1.17911923e+00,  2.81481814e+00],
#                         [ 2.22628140e+00,  1.30195391e+00,  2.79427671e+00],
#                         [ 1.89803088e+00,  1.41122890e+00,  2.82753468e+00],
#                         [ 1.59487998e+00,  1.48304808e+00,  2.98406839e+00],
#                         [ 1.37248278e+00,  1.49022830e+00,  3.13793612e+00],
#                         [ 1.09629655e+00,  1.50226641e+00, -2.79984379e+00],
#                         [ 9.33909476e-01,  1.41776860e+00, -2.50297356e+00],
#                         [ 7.94404030e-01,  1.28071511e+00, -2.23419261e+00],
#                         [ 6.92177355e-01,  1.10330057e+00, -1.90989697e+00],
#                         [ 6.54197097e-01,  8.96763921e-01, -1.57418334e+00],
#                         [ 6.70448065e-01,  7.01881707e-01, -1.34972692e+00],
#                         [ 7.63061225e-01,  5.05258143e-01, -1.13600898e+00],
#                         [ 8.86669099e-01,  3.09667647e-01, -8.42582583e-01],
#                         [ 1.02726996e+00,  1.87435761e-01, -5.72583139e-01],
#                         [ 1.20022333e+00,  1.09121621e-01, -2.76646346e-01],
#                         [ 1.41937554e+00,  8.79127607e-02, -3.65659175e-03],
#                         [ 1.64978051e+00,  1.26482233e-01,  2.57413626e-01],
#                         [ 1.98993695e+00,  2.70689189e-01,  3.36985409e-01],
#                         [ 2.31365824e+00,  3.81311357e-01,  3.29619169e-01]])

# waypoints_new = np.stack((x_points*scale_factor,y_points*scale_factor, yaw_points)).T[1:,:]
# np.save("/home/dhanesh/Masters/OuterSense/Planning_test/planner/hybrid_a_star_ws/src/Hybrid_A_Star/src/Nov10_manual_jash_100scale.npy", waypoints)


# print(x_points, y_points)
# print(waypoints, waypoints.shape)

ln1, ln2 = calculate_perpendicular_points2(waypoints)
ln1toedit_index = np.where(ln1[:,0]<1.4)
ln1toedit_index2 = np.where(ln1[:,0]>6.75)
# ln1toedit_index3 = np.where((0.9<ln1[:,0]<2.3) & (0.9<ln1[:,1]<1.2))
ln1toedit_index3 = np.where((ln1[:,0]>0.88)& (ln1[:,0]<2.3) & (ln1[:,1]>0.9)&(ln1[:,1]<1.4))


ln2toedit_index = np.where(ln2[:,0]>6.4)
ln2toedit_index2 = np.where(ln2[:,0]<1.15)
ln2toedit_index3 = np.where((ln2[:,0]>5.5)& (ln2[:,0]<6.75) & (ln2[:,1]>0.98)&(ln2[:,1]<1.45))
# ln2toedit_index3 = np.where((ln1[:,0]>1.1)& (ln1[:,0]<2.3) & (ln1[:,1]>0.9)&(ln1[:,1]<1.2))
# print(ln1[ln1toedit_index[0]][:,0])
ln1[ln1toedit_index[0]] = ln1[ln1toedit_index[0]] - np.asarray([0.15,0])
ln1[ln1toedit_index2[0]] = ln1[ln1toedit_index2[0]] + np.asarray([0.15,0])
ln1[ln1toedit_index3[0]] = ln1[ln1toedit_index3[0]] + np.asarray([0,0.20])
# ln1[ln1toedit_index4[0]] = ln1[ln1toedit_index4[0]] + np.asarray([0,0.50])
ln1toedit_index4 = np.where((ln1[:,0]>1.2)& (ln1[:,0]<1.7) & (ln1[:,1]>1.0))
ln1[ln1toedit_index4[0]] = ln1[ln1toedit_index4[0]] + np.asarray([-0.23,0.10])
ln1toedit_index5 = np.where((ln1[:,0]>4.5) & (ln1[:,1]>1.35))
ln1[ln1toedit_index5[0]] = ln1[ln1toedit_index5[0]] + np.asarray([0.0,0.10])
# print(ln1toedit_index4, ln1[ln1toedit_index4[0]])

ln2[ln2toedit_index[0]] = ln2[ln2toedit_index[0]] + np.asarray([0.15,0])
ln2[ln2toedit_index2[0]] = ln2[ln2toedit_index2[0]] - np.asarray([0.15,0])
ln2[ln2toedit_index3[0]] = ln2[ln2toedit_index3[0]] + np.asarray([0,0.22])
ln2toedit_index4 = np.where((ln2[:,0]>5.0)& (ln2[:,0]<6.5) & (ln2[:,1]>0.2)&(ln2[:,1]<0.6))
ln2[ln2toedit_index4[0]] = ln2[ln2toedit_index4[0]] + np.asarray([0,-0.10])
ln2toedit_index5 = np.where((ln2[:,1]>1.65))
ln2[ln2toedit_index5[0]] = ln2[ln2toedit_index5[0]] + np.asarray([0,0.10])
# ln2toedit_index5 = np.where((ln2[:,0]>2.9)& (ln2[:,0]<3.8) & (ln2[:,1]>1.0))
# ln2[ln2toedit_index5[0]] = ln2[ln2toedit_index5[0]] + np.asarray([0,0.10])
# print(ln1[ln1toedit_index[0]][:,0])
# print(ln1)
# print(ln2)
# plt.plot(ln1[:,0], ln1[:,1], 'b+', label="Lidar after extrinsics")
waypointsedit2 = np.where(waypoints[:,0]<1.16)
waypointsedit3 = np.where((waypoints[:,0]<1.95)& (waypoints[:,1]>1.38))
waypointsedit4 = np.where(waypoints[:,0]>6.6)
waypointsedit5 = np.where((waypoints[:,0]<1.1)& (waypoints[:,1]>0.9))
waypointsedit6 = np.where((waypoints[:,0]>5.5)& (waypoints[:,1]>1.3))
waypointsedit7 = np.where((waypoints[:,0]>4.5)& (waypoints[:,0]<5.7) &(waypoints[:,1]>1.0))
waypoints[waypointsedit2[0]] = waypoints[waypointsedit2[0]]- np.asarray([0.20,0,0])
waypoints[waypointsedit4[0]] = waypoints[waypointsedit4[0]]+ np.asarray([0.10,0,0])
waypoints[waypointsedit3[0]] = waypoints[waypointsedit3[0]]+ np.asarray([0,0.10,0.35])
waypoints[waypointsedit5[0]] = waypoints[waypointsedit5[0]]+ np.asarray([0,0,0.35])
waypoints[waypointsedit6[0]] = waypoints[waypointsedit6[0]]+ np.asarray([0,0.15,0])
waypoints[waypointsedit7[0]] = waypoints[waypointsedit7[0]]+ np.asarray([0,0.15,0])
waypointsedit8 = np.where((waypoints[:,0]>1.2)& (waypoints[:,0]<2.5) &(waypoints[:,1]>1.15))
waypoints[waypointsedit8[0]] = waypoints[waypointsedit8[0]]+ np.asarray([0,0.03,0])
waypointsedit8 = np.where((waypoints[:,0]>1.2)& (waypoints[:,0]<2.2) &(waypoints[:,1]>1.15))
waypoints[waypointsedit8[0]] = waypoints[waypointsedit8[0]]+ np.asarray([0,0.03,0])
waypoints[33] = waypoints[33]+ np.asarray([0.05,0.0,0.174])
waypoints[30] = waypoints[30]+ np.asarray([0.0,0.0,0.35])
waypoints[11] = waypoints[11]- np.asarray([0.0,0.0,0.524])
waypoints[12] = waypoints[12]- np.asarray([0.0,0.0,0.35])
waypoints[13] = waypoints[13]- np.asarray([0.0,0.0,0.174])
waypoints[3] = waypoints[3]+ np.asarray([1.2,0.5,0.174])
waypoints[4] = waypoints[4]+ np.asarray([0.0,0.0,0.174])
waypoints[2] = waypoints[2]+ np.asarray([1.0,0.4,0.174])
waypoints[1] = waypoints[1]+ np.asarray([0.55,0.25,0.174])
waypoints[0] = waypoints[0]+ np.asarray([0.28,0.15,0.174])
waypoints[20] = waypoints[20]+ np.asarray([-0.2,0.0,-0.087])
waypoints[26] = waypoints[26]+ np.asarray([-0.2,0.1,0.0])
# waypoints[16] = waypoints[16]+ np.asarray([0.0,0.0,-0.58])
waypoints[35] = waypoints[35]+ np.asarray([0.0,0.0,0.174])# can comment these
waypoints[36] = waypoints[36]+ np.asarray([0.0,0.0,0.174])# can comment these
np.save("/home/dhanesh/Masters/OuterSense/Planning_100scale/planner/hybrid_a_star_ws/src/Hybrid_A_Star/src/Nov10_manual_jash_100scale2.npy", waypoints)

t_cos = np.cos(waypoints[:,-1])
t_sin = np.sin(waypoints[:,-1])
plt.plot(waypoints[:,0], waypoints[:,1], 'r+', label="Lidar after extrinsics")
plt.quiver(waypoints[:,0], waypoints[:,1], t_cos, t_sin, color='lightblue', width = 0.001, angles = 'uv' )
plt.show()
plt.close()

plt.plot(ln1[:,0], ln1[:,1], 'b+', label="Lidar after extrinsics")
plt.plot(ln2[:,0], ln2[:,1], 'g+', label="Lidar after extrinsics")
plt.plot(waypoints[:,0], waypoints[:,1], 'r+', label="Lidar after extrinsics")
plt.quiver(waypoints[:,0], waypoints[:,1], t_cos, t_sin, color='lightblue', width = 0.001, angles = 'uv' )
plt.show()
plt.close()
road_image, white_image = fill_lanes_with_color(ln1*scale_factor, ln2*scale_factor)
# road_image = cv2.flip(road_image, 0)
# road_image = rotate(road_image, 1, resize= True, center=(0,0))*255
# print(road_image)
cv2.imshow("Road Image", road_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imshow("Road Image", white_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
# -32.964026958248915             35.39230053680539
road_image = cv2.flip(road_image, 0)
# # plt.ylim([0,500])
# plt.imshow(road_image, cmap="gray")
# # plt.scatter(waypoints[:,0]*100 -32.964026958248915, waypoints[:,1]*100+35.39230053680539)
# plt.scatter(waypoints[:,0]*100 -18, waypoints[:,1]*100+35)
# plt.show()

# print(gps_pose_array, gps_pose_array.shape)
# plt.plot(gps_pose_array[::20,1], gps_pose_array[::20,2], 'b+', label="Lidar after extrinsics")
# plt.quiver(gps_pose_array[::20,1], gps_pose_array[::20,2], t_cos, t_sin, color='lightblue', width = 0.001, angles = 'uv' )
# plt.show()
# plt.close()

# plt.plot(gps_pose_array2[:,1], gps_pose_array2[:,2], 'r+', label="Lidar after extrinsics")
# plt.show()
# plt.close()

# waypoints_new = np.asarray([[ 5.2005949 ,  0.19620596],
#                             [ 4.87930918,  0.34763497],
#                             [ 4.40095568,  0.44742849],
#                             [ 3.88095665,  0.59572166],
#                             [ 3.45538354,  0.76515901],
#                             [ 3.02898049,  0.95066833],
#                             [ 2.77563643,  1.0992583 ],
#                             [ 2.21085095,  1.37062979],
#                             [ 1.77274978,  1.52956665],
#                             [ 1.34455562,  1.63030851],
#                             [ 0.917108  ,  1.51482701],
#                             [ 0.88623071,  1.48791492],
#                             [ 0.65281624,  0.80843246],
#                             [ 0.69693482,  0.22784147],
#                             [ 1.00452292, -0.01675648],
#                             [ 1.11954021, -0.13199419],
#                             [ 1.11954021, -0.13199419],
#                             [ 2.05487251,  0.22211984],
#                             [ 2.62144804,  0.52822149],
#                             [ 2.8829987 ,  0.59049785],
#                             [ 3.53598499,  0.82032979],
#                             [ 4.04759264,  0.97951627],
#                             [ 4.44880199,  1.10316765],
#                             [ 4.8867588 ,  1.2266897 ],
#                             [ 5.41461802,  1.38030207],
#                             [ 5.83995581,  1.50989068],
#                             [ 5.89020205,  1.59876108],
#                             [ 6.58808517,  1.69088006],
#                             [ 7.00521994,  1.53822482],
#                             [ 7.2296629 ,  1.23177588],
#                             [ 7.2296629 ,  0.75846814],
#                             [ 7.00521994,  0.19620596],
#                             [ 6.58808517, -0.13199419],
#                             [ 5.83995581,  0.05059068]])
# theta =[]
# for i in range(waypoints_new.shape[0]-1):
#     y2 = waypoints_new[i+1,1]
#     y1 = waypoints_new[i,1]
#     x2 = waypoints_new[i+1,0]
#     x1 = waypoints_new[i,0]
#     if (x2-x1 == 0):
#         x2 = x1+0.0000001
#     t = np.arctan((y2-y1)/(x2-x1))
#     theta.append(t)
# t_n = np.arctan((waypoints_new[0,1]-waypoints_new[-1,1])/(waypoints_new[0,0]-waypoints_new[-1,0]))
# theta.append(t_n)
# waypoints_new = np.hstack((waypoints_new, np.asarray(theta).reshape(-1,1)))
# i_cos = np.cos(waypoints_new[:,-1])
# i_sin = np.sin(waypoints_new[:,-1])
# # print(waypoints_new)
# # plt.plot(waypoints_new[:,0], waypoints_new[:,1], 'r+', label="Lidar after extrinsics")
# # plt.quiver(waypoints_new[:,0], waypoints_new[:,1], i_cos, i_sin, color='lightblue', width = 0.001, angles = 'uv' )
# # plt.show()
# # plt.close()

# ln1, ln2 = calculate_perpendicular_points(waypoints_new)
# # print("############################################")
# print(ln1)
# print(ln2)
# plt.plot(waypoints_new[:,0], waypoints_new[:,1], 'r+', label="Lidar after extrinsics")


# plt.plot(ln1[:,0], ln1[:,1], 'b+', label="Lidar after extrinsics")
# plt.plot(ln2[:,0], ln2[:,1], 'g+', label="Lidar after extrinsics")
# plt.show()
# plt.close()

# road_image = create_road_image(ln1*100, ln2*100)
# cv2.imshow("Road Image", road_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# new_road_image = resize_image_to_tenth(road_image)
# cv2.imshow("Road Image", new_road_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# cv2.imwrite("maps/figure8_track3.png", new_road_image)

# cv2.imwrite("maps/figure8_track35.png", road_image)
cv2.imwrite("maps/figure8_track36.png", road_image)
# cv2.imwrite("maps/figure8_track29.png", white_image)

# 30.3822394 -48.199419 757.96629 201.956163
