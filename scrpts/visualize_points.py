#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import numpy as np
from numpy import linalg as la
import tf
import os
import csv
import rospkg

rospack = rospkg.RosPack()
package_path = rospack.get_path('f1tenth_gym_ros')
file_path = rospy.get_param('CSV_path')
csv_path = package_path + file_path

with open(csv_path) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

rospy.init_node('marker_node')

waypoints_pub = rospy.Publisher('/waypoints_points', MarkerArray, queue_size="1")

while not rospy.is_shutdown():
    markerArray = MarkerArray()
    for i in range(len(path_points)):
        if i % 1 == 0:
            point = path_points[i]
            x = float(point[0])
            y = float(point[1])
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0

            markerArray.markers.append(marker)
    # Renumber the marker IDs
	id = 0
	for m in markerArray.markers:
		m.id = id
		id += 1

    # Publish the MarkerArray
    waypoints_pub.publish(markerArray)

    rospy.sleep(2.0)
