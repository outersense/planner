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




bagfile_path = "/home/dhanesh/Masters/OuterSense/Planning/rosbags/oct9"
bagfile_name = "1.bag"

topic_gps = "/rccar_pose"
topic_car1_world = "/car1/world_pose"


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
    
    cv2.fillPoly(road_image, [points_shifted.astype(int)], (255, 255, 255))

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

# Initialising and errors
gps_msg_count = bag_ego.get_message_count(topic_filters= topic_gps)
gps_pose_array = np.zeros(shape=(gps_msg_count,5))
if gps_msg_count ==0:
    print("no gps dynamic global pose availabe")

# gps_msg_count2 = bag_ego.get_message_count(topic_filters= topic_car1_world)
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

    # if topic == topic_car1_world:
    #     heading_degree_gps2, location_gps2, pose_gps2 = get_carworld_pose(msg)
    #     # print(location_gps)
    #     gps_pose_array2[gps_msgs2] = pose_gps2
    #     gps_msgs2 += 1
yaw_array = gps_pose_array[:,4]#*180/PI
x_points = gps_pose_array[::20,1]
y_points = gps_pose_array[::20,2]
yaw_points = yaw_array[::20]
waypoints = np.stack((x_points,y_points, yaw_points)).T

t_cos = np.cos(yaw_points)
t_sin = np.sin(yaw_points)

# print(x_points, y_points)
print(waypoints, waypoints.shape)

# print(gps_pose_array, gps_pose_array.shape)
# plt.plot(gps_pose_array[::20,1], gps_pose_array[::20,2], 'b+', label="Lidar after extrinsics")
# plt.quiver(gps_pose_array[::20,1], gps_pose_array[::20,2], t_cos, t_sin, color='lightblue', width = 0.001, angles = 'uv' )
# plt.show()
# plt.close()

# plt.plot(gps_pose_array2[:,1], gps_pose_array2[:,2], 'r+', label="Lidar after extrinsics")
# plt.show()
# plt.close()

waypoints_new = np.asarray([[ 5.2005949 ,  0.19620596],
                            [ 4.87930918,  0.34763497],
                            [ 4.40095568,  0.44742849],
                            [ 3.88095665,  0.59572166],
                            [ 3.45538354,  0.76515901],
                            [ 3.02898049,  0.95066833],
                            [ 2.77563643,  1.0992583 ],
                            [ 2.21085095,  1.37062979],
                            [ 1.77274978,  1.52956665],
                            [ 1.34455562,  1.63030851],
                            [ 0.917108  ,  1.51482701],
                            [ 0.88623071,  1.48791492],
                            [ 0.65281624,  0.80843246],
                            [ 0.69693482,  0.22784147],
                            [ 1.00452292, -0.01675648],
                            [ 1.11954021, -0.13199419],
                            [ 1.11954021, -0.13199419],
                            [ 2.05487251,  0.22211984],
                            [ 2.62144804,  0.52822149],
                            [ 2.8829987 ,  0.59049785],
                            [ 3.53598499,  0.82032979],
                            [ 4.04759264,  0.97951627],
                            [ 4.44880199,  1.10316765],
                            [ 4.8867588 ,  1.2266897 ],
                            [ 5.41461802,  1.38030207],
                            [ 5.83995581,  1.50989068],
                            [ 5.89020205,  1.59876108],
                            [ 6.58808517,  1.69088006],
                            [ 7.00521994,  1.53822482],
                            [ 7.2296629 ,  1.23177588],
                            [ 7.2296629 ,  0.75846814],
                            [ 7.00521994,  0.19620596],
                            [ 6.58808517, -0.13199419],
                            [ 5.83995581,  0.05059068]])
theta =[]
for i in range(waypoints_new.shape[0]-1):
    y2 = waypoints_new[i+1,1]
    y1 = waypoints_new[i,1]
    x2 = waypoints_new[i+1,0]
    x1 = waypoints_new[i,0]
    if (x2-x1 == 0):
        x2 = x1+0.0000001
    t = np.arctan((y2-y1)/(x2-x1))
    theta.append(t)
t_n = np.arctan((waypoints_new[0,1]-waypoints_new[-1,1])/(waypoints_new[0,0]-waypoints_new[-1,0]))
theta.append(t_n)
waypoints_new = np.hstack((waypoints_new, np.asarray(theta).reshape(-1,1)))
i_cos = np.cos(waypoints_new[:,-1])
i_sin = np.sin(waypoints_new[:,-1])
# print(waypoints_new)
# plt.plot(waypoints_new[:,0], waypoints_new[:,1], 'r+', label="Lidar after extrinsics")
# plt.quiver(waypoints_new[:,0], waypoints_new[:,1], i_cos, i_sin, color='lightblue', width = 0.001, angles = 'uv' )
# plt.show()
# plt.close()

pp1, pp2 = calculate_perpendicular_points(waypoints_new)
# print("############################################")
# print(pp1)
# print(pp2)
# plt.plot(waypoints_new[:,0], waypoints_new[:,1], 'r+', label="Lidar after extrinsics")


# road_image = create_road_image(pp1*100, pp2*100)
# cv2.imshow("Road Image", road_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# first half path 
#  [1.4, 6.5, 7.0, 7.1, 7.2, 6.95, 5.5, 5.0],  # 6.0],  # 6.7],
#             [0.2, 1.5, 1.3, 1.1, 0.8, 0.2, 0.2, 0.2],

# 2nd half path
# [4.8, 1.4, 0.9, 0.7, 0.6, 0.9, 1.4],  # 6.0],  # 6.7],
#             [0.0, 1.7, 1.5, 1.2, 0.8, 0.2, 0.2],


# function to create path
# def compute_path_from_wp(start_xp, start_yp, step=0.0001):
#     """
#     Computes a reference path given a set of waypoints
#     """
#     final_xp = []
#     final_yp = []
#     delta = step  # [m]
#     for idx in range(len(start_xp) - 1):
#         section_len = np.sum(
#             np.sqrt(
#                 np.power(np.diff(start_xp[idx : idx + 2]), 2)
#                 + np.power(np.diff(start_yp[idx : idx + 2]), 2)
#             )
#         )
#         # print(delta)
#         interp_range = np.linspace(0, 1, np.floor(section_len / delta).astype(int))
#         fx = interp1d(np.linspace(0, 1, 2), start_xp[idx : idx + 2], kind=1)
#         fy = interp1d(np.linspace(0, 1, 2), start_yp[idx : idx + 2], kind=1)
#         # watch out to duplicate points!

#         final_xp = np.append(final_xp, fx(interp_range)[1:])
#         # print(section_len)
#         # print(section_len / delta)
#         final_yp = np.append(final_yp, fy(interp_range)[1:])

#     dx = np.append(0, np.diff(final_xp))
#     dy = np.append(0, np.diff(final_yp))

#     theta = np.arctan2(dy, dx)
#     return np.vstack((final_xp, final_yp, theta))




# [[5.34981401 0.51280295]
#  [4.9507869  0.69025858]
#  [4.49694171 0.78400936]
#  [4.0104216  0.92089673]
#  [3.59501154 1.08610141]
#  [3.20605143 1.25257209]
#  [2.92721689 1.41473154]
#  [2.33021381 1.69964728]
#  [1.85290623 1.87026438]
#  [1.25327053 1.96819463]
#  [0.68714259 1.77867528]
#  [0.5552168  1.60162416]
#  [1.00181009 0.83495218]
#  [0.91477748 0.50178412]
#  [1.2522471  0.23049386]
#  [1.11954021 0.21800581]
#  [0.99561537 0.19533231]
#  [1.88850691 0.53005244]
#  [2.54037791 0.86870296]
#  [2.76679666 0.92064492]
#  [3.43199999 1.15452602]
#  [3.9445085  1.31399147]
#  [4.35379395 1.44002589]
#  [4.78896216 1.56274895]
#  [5.31261206 1.71510766]
#  [5.53528106 1.68215007]
#  [5.84440014 1.94575126]
#  [6.70836995 2.01956163]
#  [7.28758742 1.74503054]
#  [7.5796629  1.23177595]
#  [6.90460406 0.88822463]
#  [6.7887984  0.47127271]
#  [6.67106886 0.20802597]
#  [5.91767849 0.39185185]]
# [[ 5.05137579e+00 -1.20391035e-01]
#  [ 4.80783146e+00  5.01136301e-03]
#  [ 4.30496965e+00  1.10847621e-01]
#  [ 3.75149170e+00  2.70546590e-01]
#  [ 3.31575554e+00  4.44216614e-01]
#  [ 2.85190955e+00  6.48764567e-01]
#  [ 2.62405597e+00  7.83785062e-01]
#  [ 2.09148809e+00  1.04161230e+00]
#  [ 1.69259333e+00  1.18886892e+00]
#  [ 1.43584071e+00  1.29242239e+00]
#  [ 1.14707341e+00  1.25097874e+00]
#  [ 1.21724462e+00  1.37420568e+00]
#  [ 3.03822394e-01  7.81912736e-01]
#  [ 4.79092159e-01 -4.61011789e-02]
#  [ 7.56798744e-01 -2.64006824e-01]
#  [ 1.11954021e+00 -4.81994190e-01]
#  [ 1.24346505e+00 -4.59320685e-01]
#  [ 2.22123811e+00 -8.58127620e-02]
#  [ 2.70251817e+00  1.87740017e-01]
#  [ 2.99920074e+00  2.60350784e-01]
#  [ 3.63996999e+00  4.86133561e-01]
#  [ 4.15067678e+00  6.45041069e-01]
#  [ 4.54381003e+00  7.66309413e-01]
#  [ 4.98455544e+00  8.90630453e-01]
#  [ 5.51662398e+00  1.04549648e+00]
#  [ 6.14463056e+00  1.33763129e+00]
#  [ 5.93600396e+00  1.25177090e+00]
#  [ 6.46780039e+00  1.36219849e+00]
#  [ 6.72285246e+00  1.33141910e+00]
#  [ 6.87966290e+00  1.23177581e+00]
#  [ 7.55472174e+00  6.28711646e-01]
#  [ 7.22164148e+00 -7.88607909e-02]
#  [ 6.50510148e+00 -4.72014347e-01]
#  [ 5.76223313e+00 -2.90670489e-01]]

ln1 = np.asarray([[5.34981401, 0.51280295],
                  [4.9507869 , 0.69025858],
                  [4.49694171, 0.78400936],
                  [4.0104216 , 0.92089673],
                  [3.59501154, 1.08610141],
                  [3.20605143, 1.25257209],
                  [2.92721689, 1.41473154],
                  [2.33021381, 1.69964728],
                  [1.85290623, 1.87026438],
                  [1.25327053, 1.96819463],
                  [0.68714259, 1.77867528],
                  [0.5552168 , 1.60162416],
                  [ 3.03822394e-01,  7.81912736e-01],
                  [ 4.79092159e-01, -4.61011789e-02],
                  [ 7.56798744e-01, -2.64006824e-01],
                  [ 1.11954021e+00, -4.81994190e-01],
                  [ 1.24346505e+00, -4.59320685e-01],
                  [ 2.22123811e+00, -8.58127620e-02],
                  [ 2.70251817e+00,  1.87740017e-01],
                  [ 2.99920074e+00,  2.60350784e-01],
                  [ 3.63996999e+00,  4.86133561e-01],
                  [ 4.15067678e+00,  6.45041069e-01],
                  [ 4.54381003e+00,  7.66309413e-01],
                  [ 4.98455544e+00,  8.90630453e-01],
                  [ 5.51662398e+00,  1.04549648e+00],
                  [ 6.14463056e+00,  1.33763129e+00],
                  [ 5.93600396e+00,  1.25177090e+00],
                  [ 6.46780039e+00,  1.36219849e+00],
                  [ 6.72285246e+00,  1.33141910e+00],
                  [ 6.87966290e+00,  1.23177581e+00],
                  [6.90460406, 0.88822463],
                  [6.7887984 , 0.47127271],
                  [6.67106886, 0.20802597],
                  [5.91767849, 0.39185185]])

ln2 = np.asarray([[ 5.05137579e+00, -1.20391035e-01],
                  [ 4.80783146e+00,  5.01136301e-03],
                  [ 4.30496965e+00,  1.10847621e-01],
                  [ 3.75149170e+00,  2.70546590e-01],
                  [ 3.31575554e+00,  4.44216614e-01],
                  [ 2.85190955e+00,  6.48764567e-01],
                  [ 2.62405597e+00,  7.83785062e-01],
                  [ 2.09148809e+00,  1.04161230e+00],
                  [ 1.69259333e+00,  1.18886892e+00],
                  [ 1.43584071e+00,  1.29242239e+00],
                  [ 1.14707341e+00,  1.25097874e+00],
                  [ 1.21724462e+00,  1.37420568e+00],
                  [1.00181009, 0.83495218],
                  [0.91477748, 0.50178412],
                  [1.2522471 , 0.23049386],
                  [1.11954021, 0.21800581],
                  [0.99561537, 0.19533231],
                  [1.88850691, 0.53005244],
                  [2.54037791, 0.86870296],
                  [2.76679666, 0.92064492],
                  [3.43199999, 1.15452602],
                  [3.9445085 , 1.31399147],
                  [4.35379395, 1.44002589],
                  [4.78896216, 1.56274895],
                  [5.31261206, 1.71510766],
                  [5.53528106, 1.68215007],
                  [5.84440014, 1.94575126],
                  [6.70836995, 2.01956163],
                  [7.28758742, 1.74503054],
                  [7.5796629 , 1.23177595],
                  [ 7.55472174e+00,  6.28711646e-01],
                  [ 7.22164148e+00, -7.88607909e-02],
                  [ 6.50510148e+00, -4.72014347e-01],
                  [ 5.76223313e+00, -2.90670489e-01]])

plt.plot(ln1[:,0], ln1[:,1], 'b+', label="Lidar after extrinsics")
plt.plot(ln2[:,0], ln2[:,1], 'g+', label="Lidar after extrinsics")
plt.show()
plt.close()

road_image = create_road_image(ln1*100, ln2*100)
cv2.imshow("Road Image", road_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
# new_road_image = resize_image_to_tenth(road_image)
# cv2.imshow("Road Image", new_road_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# cv2.imwrite("maps/figure8_track3.png", new_road_image)
cv2.imwrite("maps/figure8_track4.png", road_image)

# 30.3822394 -48.199419 757.96629 201.956163
