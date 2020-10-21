#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
import tf
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

from f1tenth_control.msg import RaceInfo


class reactive_follow_gap:
    def __init__(self):

        drive_topic = rospy.get_param('ego_drive_topic')
        odom_topic = rospy.get_param('ego_odom_topic')
        scan_topic = rospy.get_param('ego_scan_topic')
        
        # create ROS subscribers and publishers
        self.drive_pub = rospy.Publisher(
            drive_topic, AckermannDriveStamped, queue_size=1)
        self.laser_pub = rospy.Publisher(
            '/scan_modified', LaserScan, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self.pose_callback, queue_size=10)
        self.scan_sub = rospy.Subscriber(
            scan_topic, LaserScan, self.scan_callback, queue_size=10)
        
        self.info_sub = rospy.Subscriber(
            "/race_info", RaceInfo, self.info_callback, queue_size=1)


        # Params
        self.max_scan_distance = rospy.get_param('max_scan_distance')
        self.scan_distance_to_base_link = rospy.get_param(
            'scan_distance_to_base_link')
        self.bubble_radius = rospy.get_param('bubble_radius')
        self.filter_size = rospy.get_param('smoothing_filter_size')
        self.steering_angle_reactivity = rospy.get_param(
            'steering_angle_reactivity')

        self.mode = 1
        self.current_time = 0
        self.last_time = 0
        self.last_steering_angle = 0
        self.last_error_angle = 0
        self.time_flag = 0
        self.temp_flag = False
        self.vehicle_width = 0.25

        self.desired_angle = 0
        self.yaw_real = 0
        self.position_real = 0
        self.kp = 1.2
        self.ki = 0.0
        self.kd = 0.1
        self.I_term = 0
        self.P_term = 0
        self.windup_guard = 0.4
        self.time_interval = 0.02

    def info_callback(self, msg):
        """return current system time"""
        self.current_time = msg.opp_elapsed_time


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # Lidar fov is 270 degree, 1080 beams, 0.25 degree interval
        # truncate lidar range
        ranges = np.array(ranges)
        proc_ranges = ranges[np.arange(270, 810)]
        # proc_ranges = ranges[np.arange(300, 780)]

        # smooth the lidar points
        proc_ranges = self.running_mean(proc_ranges, self.filter_size)

        # Remove high values
        idxs = np.where(proc_ranges >= self.max_scan_distance)
        proc_ranges[idxs] = self.max_scan_distance

        # idxs = np.where(proc_ranges<=self.scan_distance_to_base_link)
        # proc_ranges[idxs] = self.scan_distance_to_base_link

        return proc_ranges

    def running_mean(self, x, N):
        cumsum = np.cumsum(np.insert(x, 0, 0))
        return (cumsum[N:] - cumsum[:-N]) / float(N)

    def inflate_obstacle(self, ranges, interval, start_i, end_i):
        bubble_size = 0.6
        dis_threshold = 0.5
        idx = start_i
        
        while idx < end_i:
                if ranges[idx+1]-ranges[idx] > dis_threshold:
                    idx_size = int(
                        bubble_size/(ranges[idx]*interval))
                    for j in range(idx+1, idx+idx_size):
                        # print(min(j,len(ranges)), idx)
                        ranges[min(j,len(ranges)-1)] = ranges[idx]
                        
                    idx = idx+idx_size
                elif ranges[idx]-ranges[idx+1] > dis_threshold:
                    idx_size = int(
                        bubble_size/(ranges[idx+1]*interval))
                    for j in range(idx-idx_size, idx+1):
                        # print(min(j,len(ranges)),idx+1)
                        ranges[max(j,0)] = ranges[idx+1]
                        
                idx = idx+1

        return ranges

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        return (start_i + end_i)//2

    def zero_out_bubble(self, ranges, min_distance, min_idx, angle_increment):
        radius = 0.3
        if min_distance == 0:
            alpha = np.arcsin(0)
        elif min_distance > radius:
            alpha = np.arcsin(radius/min_distance)
        else:
            alpha = np.arcsin(1)

        min_angle = angle_increment*min_idx - alpha
        max_angle = angle_increment*min_idx + alpha

        for i in range(len(ranges)):
            point_angle = angle_increment * i
            if min_angle < point_angle < max_angle:
                ranges[i] = 0.0

        return ranges

    def set_speed(self, distance):
        """ Set speed according to the distance in front of car"""
        return (1-np.exp(-abs(distance)*0.2))*4 + 2.0

    def visualize_laser(self, ranges):
        laser_msg = LaserScan()
        laser_msg.header.stamp = rospy.Time.now()
        laser_msg.header.frame_id = "ego_racecar/laser"
        laser_msg.angle_min= -2.34999990463
        laser_msg.angle_max= 2.34999990463
        laser_msg.angle_increment= 0.00435185199603
        laser_msg.range_min= 0.0
        laser_msg.range_max= 30.0
        laser_msg.ranges = ranges
        self.laser_pub.publish(laser_msg)

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

        self.yaw_real = yaw
        self.position_real = position     


    def scan_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
    
        proc_ranges =  np.array(data.ranges)
        proc_ranges = self.running_mean(proc_ranges, self.filter_size)
        scan_min_idx = int((-1.57-data.angle_min) / data.angle_increment)
        scan_max_idx = int((1.57-data.angle_min) / data.angle_increment)

        proc_ranges = self.inflate_obstacle(
            proc_ranges, data.angle_increment, scan_min_idx, scan_max_idx)
        depth = np.max(proc_ranges)
        best_idx = np.argmax(
            proc_ranges[scan_min_idx:scan_max_idx]) + scan_min_idx + 1
        
        desired_angle = (
            best_idx*data.angle_increment+data.angle_min)



        # error_angle = -self.yaw_real + desired_angle
        # self.P_term = self.kp * error_angle
        # # self.I_term += self.ki * error_angle * self.time_interval
        # self.D_term = self.kd * (1/self.time_interval) * (error_angle - self.last_error_angle)
        
        # if (self.I_term < -self.windup_guard):
        #     self.I_term = -self.windup_guard
        # elif (self.I_term > self.windup_guard):
        #     self.I_term = self.windup_guard

        # control = self.P_term + self.I_term + self.D_term
        # self.last_error_angle = error_angle

        # control = min(max(control, -0.4189), 0.4189)
        # steering_angle = control

        speed = 5
        # steering_angle = desired_angle
        # if abs(depth) > 40 and abs(steering_angle) < 0.3: 
        #     speed = 12
        # elif abs(depth) > 30 and abs(steering_angle) < 0.2: 
        #     speed = 10 + min((abs(depth) -30) *0.2, 2)
        # elif abs(depth) > 20 and abs(steering_angle) < 0.15: 
        #     speed = 8 + min((abs(depth) -20) *0.2, 2)
        # elif abs(depth) > 10 and abs(steering_angle) < 0.1: 
        #     speed = 6 + min((abs(depth) -10) *0.2, 2)
        # elif abs(depth) > 7 and abs(steering_angle) < 0.08: 
        #     speed = 3.5 + min((abs(depth) -7) *0.5, 1.5)  
        # else:
        #     speed = 3.5

        if abs(depth) > 7 and abs(desired_angle) < 0.1: 
            speed = 5 + min((abs(depth) -7) *0.5, 7)  
        else:
            speed = 5
        speed = min(speed, 15)

        print("speed: {:.4f}, depth: {:.4f}, steer: {:.4f}".format(speed, depth, desired_angle))
          
        # yaw rate saturation
        steering_angle = desired_angle
        angle_interval = steering_angle - self.last_steering_angle        
        steering_rate = angle_interval / self.time_interval      
        steering_rate = np.clip(steering_rate, -2.5, 2.5)
        steering_angle += steering_rate * self.time_interval
        self.last_steering_angle = steering_angle
        # print(steering_rate)
        # steering_angle = steering_angle * 0.6

        self.time_flag += 1
        if self.time_flag % 50 == 0:
            self.visualize_laser(proc_ranges)
            self.time_flage = 0

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "drive"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("gap_follow_node", anonymous=True)
    rfgs = reactive_follow_gap()
    r = rospy.Rate(50) # hz
    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main(sys.argv)
