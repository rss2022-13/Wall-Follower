#!/usr/bin/env python2

import numpy as np
import scipy
import rospy
import math
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from visualization_tools import *

class WallFollower:
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    # WALL_TOPIC = "/wall"
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.controller)
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        # self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        self.previous_error = 0

    def controller(self, data):
        ranges = data.ranges
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_change = data.angle_increment

        angles = [angle_min + i*angle_change for i in range(len(ranges))]

        middle_ix = len(angles) // 2

        # Get the half of the ranges/angles lists that are relevant for desired side to follow
        if self.SIDE == -1: # LEFT
            relevant_data = ranges[:middle_ix]
            relevant_angles = angles[:middle_ix]
        else: # RIGHT
            relevant_data = ranges[middle_ix:]
            relevant_angles = angles[middle_ix:]

        # Determine how close the closest reading is
        closest_index = relevant_data.index(min(relevant_data))
        closest_point_angle = relevant_angles[closest_index]
        
        targeted_ranges = np.array(relevant_data[max(0, closest_index - 16): min(middle_ix, closest_index + 16)])
        targeted_angles = np.array(relevant_angles[max(0, closest_index - 16): min(middle_ix, closest_index + 16)])
        
        targeted_x = np.cos(targeted_angles) * targeted_ranges
        targeted_y = np.sin(targeted_angles) * targeted_ranges
        
        wall_fit = np.polyfit(targeted_x,targeted_y, 1)
        
        if (min(ranges[middle_ix-5:middle_ix+5]) > 2): # If there is no wall close to us, do the math to find where the wall is
            wall_angle = math.atan2(wall_fit[0], 1.0) 
            wall_distance = wall_fit[1] * math.cos(wall_angle)
        
        else: # Failsafe...Wall is immenent, make sharp turn
            wall_angle = - self.SIDE * math.pi / 2.0
            wall_distance = 0
            
        error = wall_distance - self.SIDE * self.DESIRED_DISTANCE
        error += wall_angle * 0.5
        
        kp = 4.0
        kd = 2.0
        derivative_error = error - self.previous_error
        angle = error * kp + derivative_error * kd

        angle = min(max(angle, -math.pi/2), math.pi/2)
        angle += wall_angle
        self.previous_error = error
        
        reaction = AckermannDriveStamped()
        reaction.header.stamp = rospy.Time.now()
        reaction.drive.speed = self.VELOCITY
        reaction.drive.steering_angle = angle
        self.pub.publish(reaction)

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
