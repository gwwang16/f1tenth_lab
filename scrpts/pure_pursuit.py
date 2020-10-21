#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
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

from utils import visualize_point, nearest_point_on_trajectory, first_point_on_trajectory_intersecting_circle


class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """

    def __init__(self):

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('f1tenth_gym_ros')
        file_path = rospy.get_param('CSV_path')
        self.csv_path = package_path + file_path

        drive_topic = rospy.get_param('ego_drive_topic')
        odom_topic = rospy.get_param('ego_odom_topic')
        scan_topic = rospy.get_param('ego_scan_topic')
        # self.max_velocity = rospy.get_param('max_velocity')

        self.safe_speed = 2.0
        self.wheelbase = 0.3302
        self.max_reacquire = 10.
        self.lookahead_distance = 1.5
        self.lookahead_distance_ = 1.5
        self.max_velocity = 7.5
        self.min_speed = 2.5

        self.read_waypoints(self.csv_path)

        # create ROS subscribers and publishers
        self.drive_pub = rospy.Publisher(
            drive_topic, AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self.pose_callback, queue_size=10)
        # self.scan_sub = rospy.Subscriber(
        #     scan_topic, LaserScan, self.scan_callback, queue_size=10)

        # Publisher for the goal point
        self.goal_pub = rospy.Publisher(
            '/lookahead_point', MarkerArray, queue_size="1")

    def read_waypoints(self, csv_path):
        with open(csv_path) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
            self.path_points = path_points
            self.waypoints = np.array(
                [(float(pt[0]), float(pt[1]), float(pt[2]), float(pt[3])) for pt in path_points])

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points_x = np.asarray(
            [float(point[0]) for point in path_points])
        self.path_points_y = np.asarray(
            [float(point[1]) for point in path_points])

        # list of xy pts
        self.xy_points = np.hstack((self.path_points_x.reshape(
            (-1, 1)), self.path_points_y.reshape((-1, 1)))).astype('double')

    def _get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
        wpts = waypoints[:, 0:2]
        nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(
            position, wpts)

        if nearest_dist < lookahead_distance:
            lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(
                position, lookahead_distance, wpts, i+t, wrap=True)
            if i2 == None:
                return None
            current_waypoint = np.empty(waypoints[i2, :].shape)
            # x, y
            current_waypoint[0:2] = waypoints[i2, 0:2]
            # theta
            current_waypoint[3] = waypoints[i2, 3]
            # speed
            current_waypoint[2] = waypoints[i2, 2]
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            return waypoints[i, :]
        else:
            return None

    def get_actuation(self, pose_theta, lookahead_point, position, lookahead_distance, wheelbase):
        dist = (lookahead_point[:2] - position).astype('double')
        # goalx_veh = (dist[0] * np.cos(pose_theta)) + \
        #     (dist[1] * np.sin(pose_theta))
        goaly_veh = (-dist[0] * np.sin(pose_theta)) + \
            (dist[1] * np.cos(pose_theta))

        L = np.sqrt((lookahead_point[0]-position[0])
                    ** 2 + (lookahead_point[1]-position[1])**2)

        arc = 2 * goaly_veh/(L**2)
        steering_angle = 0.8 * arc

        # steering_angle = np.clip(steering_angle, -0.4, 0.4)
        speed = self.set_speed(steering_angle)

        return speed, steering_angle

    def pose_callback(self, pose_msg):
        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                               pose_msg.pose.pose.orientation.y,
                               pose_msg.pose.pose.orientation.z,
                               pose_msg.pose.pose.orientation.w])

        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = np.double(euler[2])
        # finding the distance of each way point from the current position
        position = [pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y]

        lookahead_point = self._get_current_waypoint(
            self.waypoints, self.lookahead_distance, position, yaw)
        visualize_point(lookahead_point, self.goal_pub)

        if lookahead_point is None:
            return self.safe_speed, 0.0
        speed, steering_angle = self.get_actuation(
            yaw, lookahead_point, position, self.lookahead_distance, self.wheelbase)

        # Adaptive lookahead distance
        self.lookahead_distance = self.lookahead_distance_ * \
            (1.0 + speed/self.max_velocity*0.45)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "/drive"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def set_speed(self, angle):
        return np.exp(-abs(angle))*5 + self.min_speed


def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main()
