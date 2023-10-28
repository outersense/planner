#!/usr/bin/env python3


#  This is node that listens to fused data that is of pose stampped message type
# header: 
#   seq: 2409
#   stamp: 
#     secs: 1697571825
#     nsecs:  84017754
#   frame_id: "world"
# child_frame_id: "car2/base_link"
# pose: 
#   pose: 
#     position: 
#       x: 4.81719369974616
#       y: 0.20964775495888713
#       z: 8.515506985626891e-17
#     orientation: 
#       x: -1.0645422195947236e-18
#       y: 1.1655235856498888e-16
#       z: 0.9891147136329634
#       w: 0.1471464687812136
#   covariance: [0.10555161138815394, 3.062115609911795e-09, 2.7355735951529573e-25, -4.650460029946241e-25, 3.83505836792795e-21, -1.6506096026961142e-14, 3.062115609912007e-09, 0.10555163843494872, 4.385836841402432e-24, -1.4945889212464688e-24, -1.1017906097304037e-21, -5.33312875232135e-14, 2.7355735950890085e-25, -2.231810010516407e-24, 1.5558031201453346e-06, -2.1008079361714568e-19, -7.917988555775595e-15, 1.663223160174368e-21, 1.2769843904344625e-23, -1.4946895457664422e-24, -2.1008079361714366e-19, 1.4066593508457507e-06, 6.383942135102298e-17, 1.1471824004974714e-32, 3.835058367821831e-21, -1.1050993325303647e-21, -7.917988562393027e-15, 6.383942135102298e-17, 1.406659350783465e-06, -1.2472962453739365e-21, -1.6506096028615503e-14, -5.33312875232135e-14, 1.6632231601754195e-21, -2.1488570665025156e-29, -1.247296223872762e-21, 3.4066173070679447e-06]
# twist: 
#   twist: 
#     linear: 
#       x: 0.49894534218037734
#       y: -1.0973387133042312e-10
#       z: -8.46226250344752e-17
#     angular: 
#       x: -9.33038540958811e-18
#       y: -8.544765695008065e-17
#       z: -0.0033367921510789583
#   covariance: [0.0005010088481122983, -1.5796431104794825e-21, 6.78012999858353e-26, -1.7462633274307452e-28, -1.9776938887050552e-27, 5.988393154936348e-22, -1.5796431104794818e-21, 0.0005010088481119932, 5.0040238497350123e-29, 4.210623271354027e-33, 1.7720143016089391e-31, 2.553776130835845e-22, -3.559312113967227e-26, 4.9251377592134526e-29, 1.4663689031948455e-06, 6.308286649009168e-23, -9.854010218034441e-19, 1.7570132866342285e-30, -1.746263327430745e-28, 4.210623271353815e-33, 6.308286649009173e-23, 1.2347785881310081e-06, 1.2296771546825321e-24, 4.086813419138285e-28, -1.977693912779225e-27, 1.7717735603371286e-31, -9.854010218539304e-19, 1.2296771546825329e-24, 1.2347785881309926e-06, -2.8090322806766235e-29, 5.988910143312232e-22, 2.5537761352731877e-22, 1.7816651626233804e-30, 4.1951093051238335e-30, -2.828753807615544e-29, 0.0004002122586454154]
# ---

#  it gets the car1 values x and y and publishes in format initial
# header: 
#   seq: 0
#   stamp: 
#     secs: 1697571817
#     nsecs: 344560695
#   frame_id: "world"
# pose: 
#   pose: 
#     position: 
#       x: 84.54855346679688
#       y: 29.91866683959961
#       z: 0.0
#     orientation: 
#       x: 0.0
#       y: 0.0
#       z: 0.18952627488212456
#       w: 0.9818756495245746
#   covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
# ---

#  once obtained xy and theta it checks for nearest neighbour and them finds a goal states xy and theta and publishes it in the format
# and goal as
# header: 
#   seq: 1
#   stamp: 
#     secs: 1697571823
#     nsecs: 370301804
#   frame_id: "world"
# pose: 
#   position: 
#     x: 105.09276580810547
#     y: 46.70957946777344
#     z: 0.0
#   orientation: 
#     x: 0.0
#     y: 0.0
#     z: 0.38412213944847984
#     w: 0.9232822872694584
# ---


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

def odom_callback(msg):
    # Create a PoseStamped message
    pose_cov_msg = PoseWithCovarianceStamped()

    # Preserve the timestamp from the Odometry message
    pose_cov_msg.header.stamp = msg.header.stamp
    pose_cov_msg.header.frame_id = "world"  # Set the frame_id as "world"

    # Extract the position and orientation data from the Odometry message
    pose_cov_msg.pose.pose.position.x = msg.pose.pose.position.x*100-30
    pose_cov_msg.pose.pose.position.y = msg.pose.pose.position.y*100+48
    pose_cov_msg.pose.pose.position.z = 0.0
    pose_cov_msg.pose.pose.orientation = msg.pose.pose.orientation

    # Set the covariance matrix from the Odometry message
    pose_cov_msg.pose.covariance = msg.pose.covariance

    # Publish the PoseWithCovarianceStamped message
    pose_cov_publisher.publish(pose_cov_msg)

    

if __name__ == '__main__':
    rospy.init_node('publish_curr_pose_and_goal_pose')

    # Subscriber for nav_msgs/Odometry messages
    rospy.Subscriber('/car2/fused', Odometry, odom_callback)  # Replace 'your_odom_topic' with the actual topic name

    # Publisher for geometry_msgs/PoseStamped messages
    pose_cov_publisher = rospy.Publisher('/car2/planner_curr_pos', PoseWithCovarianceStamped, queue_size=10)  # Replace 'pose_stamped_topic' with the desired topic name

    rospy.spin()