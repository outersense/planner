#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import random

def publish_static_obstacles():
    # Initialize the ROS node
    rospy.init_node("static_obstacles_publisher", anonymous=True)
    
    # Create a publisher for the "static_obstacles" topic
    pub = rospy.Publisher("/static_obstacles", Int32MultiArray, queue_size=10)
    
    # Set the publishing rate to 20 Hz
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        # Create an Int32MultiArray message to store obstacle data
        obstacles = Int32MultiArray()
        
        for i in range(1, 6):
            # obstacle_data = [i, random.randint(0, 50), random.randint(0, 30), 0, random.choice([0, 2])]
            obstacle_data = [i, i*6+10, i*4+10, 0, 0]
            obstacles.data.extend(obstacle_data)
        
        # Publish the Int32MultiArray message
        pub.publish(obstacles)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_static_obstacles()
    except rospy.ROSInterruptException:
        pass
