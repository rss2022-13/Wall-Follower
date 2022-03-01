#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = "/vesc/ackermann_cmd_mux/input/navigation"
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    integral = 0
    previous_error = 0
    kp = rospy.get_param("wall_follower/kp")
    kd = rospy.get_param("wall_follower/kd")
    
    def __init__(self):

        #SUBSCRIBER
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.line_pub = rospy.Publisher("/wall", Marker, queue_size=1)
        rospy.loginfo(self.SIDE)
        
    def callback(self, data):
        x = data.ranges * np.cos(np.linspace(data.angle_min, data.angle_max, len(data.ranges)))
        y = data.ranges * np.sin(np.linspace(data.angle_min, data.angle_max, len(data.ranges)))

        length = x.size
        side = ""

        if self.SIDE == -1:
            x = x[:length//2]
            y = y[:length//2]
            x = x[np.array(data.ranges[:length//2]) < 3.0*self.DESIRED_DISTANCE]
            y = y[np.array(data.ranges[:length//2]) < 3.0*self.DESIRED_DISTANCE]
        else:
            x = x[length//2:]
            y = y[length//2:]
            x = x[np.array(data.ranges[length[1]//2:]) < 3.0*self.DESIRED_DISTANCE]
            y = y[np.array(data.ranges[length[1]//2:]) < 3.0*self.DESIRED_DISTANCE]

        
        
        ##VISUALIATION data to consider, green
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser", color=(0,0,1))
        #return
        wall = np.polyfit(x,y,1)

        a = wall[0]
        b = wall[1]

        #VISUALIATION wall black
        #VisualizationTools.plot_line(x, a*x+b, self.line_pub, frame="/laser", color = (1,1,1))
        
        theta = np.arctan(a)

        
        if self.SIDE == -1:
            error = self.DESIRED_DISTANCE + b*np.cos(theta)
        else:
            error = -self.DESIRED_DISTANCE + b*np.cos(theta)
        rospy.loginfo(error)

        P = self.kp #proportion
        I = 0 #integral
        D = self.kd #derivative


        dt = 1.0/50
        self.integral += error * dt

        derivative = (error - self.previous_error)/dt
        self.previous_error = error
        
        pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = "base_link"
        msg.drive.speed = self.VELOCITY
        msg.drive.acceleration = 0
        msg.drive.steering_angle = P*error + D*derivative + I*self.integral

        msg.drive.steering_angle_velocity = 0

        pub.publish(msg)
        

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
