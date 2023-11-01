# #!/usr/bin/env python

# import rospy
# import rosbag
# import matplotlib.pyplot as plt
# from nav_msgs.msg import Odometry
# import math

# class RosBagParser:
#     def __init__(self):
#         rospy.init_node('bag_parser_node', anonymous=True)
#         self.plot_arrows = True  # Set this to False to disable arrow plotting

#     def parse_bag(self, bag_file):
#         x_data = []
#         y_data = []
#         heading_data = []

#         with rosbag.Bag(bag_file, 'r') as bag:
#             for topic, msg, timestamp in bag.read_messages(topics='/car1/fused'):
#                 x_data.append(msg.pose.pose.position.x)
#                 y_data.append(msg.pose.pose.position.y)
#                 orientation = msg.pose.pose.orientation
#                 heading = 2 * math.atan2(orientation.z, orientation.w)
#                 heading_data.append(heading)

#         self.create_lane(x_data, y_data, heading_data)
#         self.plot_data(x_data, y_data, heading_data)

#     def create_lane(self, x_data, y_data, heading_data):
#         lane_x_data = []
#         lane_y_data = []

#         lane_width = 0.3  # 1.5 meters on each side

#         for x, y, heading in zip(x_data, y_data, heading_data):
#             # Calculate points 1.5 meters to the left and right of the centerline
#             x_left = x + lane_width * math.cos(heading + math.pi / 2)
#             y_left = y + lane_width * math.sin(heading + math.pi / 2)
#             x_right = x + lane_width * math.cos(heading - math.pi / 2)
#             y_right = y + lane_width * math.sin(heading - math.pi / 2)

#             lane_x_data.append([x_left, x_right])
#             lane_y_data.append([y_left, y_right])

#         return lane_x_data, lane_y_data

#     def plot_data(self, x_data, y_data, heading_data):
#         plt.figure()
#         if self.plot_arrows:
#             plt.quiver(x_data, y_data, [math.cos(h) for h in heading_data], [math.sin(h) for h in heading_data], angles='xy', scale_units='xy', scale=5, color='b', label='Heading Arrows')
#         else:
#             plt.scatter(x_data, y_data, c='b', marker='o', label='Odometry Points')
#         lane_x_data, lane_y_data = self.create_lane(x_data, y_data, heading_data)
#         plt.plot(lane_x_data, lane_y_data, 'r--', label='Lane Boundary')
#         plt.xlabel('X')
#         plt.ylabel('Y')
#         plt.title('Odometry Data with Lane')
#         plt.grid(True)
#         plt.legend()
#         plt.show()

#     def run(self):
#         bag_file = '/home/jash/outersense-hybrid-astar/planner/rosbags/oct17/1_car_1_loop.bag'
#         self.parse_bag(bag_file)

# if __name__ == '__main__':
#     try:
#         parser = RosBagParser()
#         parser.run()
#     except rospy.ROSInterruptException:
#         pass



import rospy
import rosbag
import matplotlib.pyplot as plt
import cv2
import numpy as np
from nav_msgs.msg import Odometry
import math

class RosBagParser:
    def __init__(self):
        rospy.init_node('bag_parser_node', anonymous=True)
        self.plot_arrows = True  # Set this to False to disable arrow plotting

    def parse_bag(self, bag_file):
        x_data = []
        y_data = []
        heading_data = []

        with rosbag.Bag(bag_file, 'r') as bag:
            for topic, msg, timestamp in bag.read_messages(topics='/car1/fused'):
                x_data.append(msg.pose.pose.position.x)
                y_data.append(msg.pose.pose.position.y)
                orientation = msg.pose.pose.orientation
                heading = 2 * math.atan2(orientation.z, orientation.w)
                heading_data.append(heading)

        self.create_lane(x_data, y_data, heading_data)
        self.plot_data(x_data, y_data, heading_data)
        plt.savefig('output_image.png')  # Save the plotted image

        self.find_intersection_and_plot('output_image.png')  # Find intersections and plot circles

    def create_lane(self, x_data, y_data, heading_data):
        lane_x_data = []
        lane_y_data = []

        lane_width = 0.3  # 1.5 meters on each side

        for x, y, heading in zip(x_data, y_data, heading_data):
            x_left = x + lane_width * math.cos(heading + math.pi / 2)
            y_left = y + lane_width * math.sin(heading + math.pi / 2)
            x_right = x + lane_width * math.cos(heading - math.pi / 2)
            y_right = y + lane_width * math.sin(heading - math.pi / 2)

            lane_x_data.append([x_left, x_right])
            lane_y_data.append([y_left, y_right])

        return lane_x_data, lane_y_data

    def plot_data(self, x_data, y_data, heading_data):
        plt.figure()
        # if self.plot_arrows:
        #     plt.quiver(x_data, y_data, [math.cos(h) for h in heading_data], [math.sin(h) for h in heading_data], angles='xy', scale_units='xy', scale=5, color='b', label='Heading Arrows')
        # else:
        #     plt.scatter(x_data, y_data, c='b', marker='o', label='Odometry Points')
        lane_x_data, lane_y_data = self.create_lane(x_data, y_data, heading_data)
        plt.plot(lane_x_data, lane_y_data, 'r', label='Lane Boundary')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Odometry Data with Lane')
        plt.grid(True)
        plt.legend()

    def find_intersection_and_plot(self, image_file):
        img = cv2.imread(image_file)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if len(contour) >= 6:
                ellipse = cv2.fitEllipse(contour)
                cv2.ellipse(img, ellipse, (0, 255, 0), 2)

        cv2.imshow('Intersections', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def run(self):
        bag_file = '/home/jash/outersense-hybrid-astar/planner/rosbags/oct17/1_car_1_loop.bag'
        self.parse_bag(bag_file)

if __name__ == '__main__':
    try:
        parser = RosBagParser()
        parser.run()
    except rospy.ROSInterruptException:
        pass
