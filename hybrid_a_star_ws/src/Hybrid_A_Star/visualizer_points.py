#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys

def read_points(file_path):
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            x, y, theta = map(float, line.strip().split()[:3])
            points.append((x, y, theta))
    return points

def create_marker(points):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.scale.x = 10
    marker.scale.y = 10
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0

    for x, y, _ in points:
        p = Point(x, y, 0)
        marker.points.append(p)

    return marker

def create_text_marker(point, id, text):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.id = id
    marker.scale.z = 8  # Text size
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 1  # Height of the text above the point
    marker.text = str(text)
    return marker

def main(file_path):
    rospy.init_node('points_visualizer', anonymous=True)
    points_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    text_pub = rospy.Publisher('visualization_marker_text', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    points = read_points(file_path)
    points_marker = create_marker(points)

    while not rospy.is_shutdown():
        points_pub.publish(points_marker)

        for i, point in enumerate(points):
            text_marker = create_text_marker(point, i, i)
            text_pub.publish(text_marker)

        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python script.py path_to_file.txt")
    else:
        file_path = sys.argv[1]
        main(file_path)
