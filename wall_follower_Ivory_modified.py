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
    ki = 0
    
    def __init__(self):

        #SUBSCRIBER
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.line_pub = rospy.Publisher("/wall", Marker, queue_size=1)
        self.front_line_pub = rospy.Publisher("/front_wall", Marker, queue_size=1)
        #rospy.loginfo(self.SIDE)
        
    def callback(self, data):
        x = data.ranges * np.cos(np.linspace(data.angle_min, data.angle_max, len(data.ranges)))
        y = data.ranges * np.sin(np.linspace(data.angle_min, data.angle_max, len(data.ranges)))
        length = x.size

        # Number of indices for angle window for front wall linear fit
        ang_window = int((np.pi/8)/data.angle_increment) 
        side = ""
        
        front_x = x[length//2-ang_window:length//2+ang_window]
        front_y = y[length//2-ang_window:length//2+ang_window]

        if self.SIDE == -1:
            x = x[:length//2-ang_window]
            y = y[:length//2-ang_window]
            #x = x[np.array(data.ranges[:length//2]) < 3.0*self.DESIRED_DISTANCE]
            #y = y[np.array(data.ranges[:length//2]) < 3.0*self.DESIRED_DISTANCE]
        else:
            x = x[length//2+ang_window:]
            y = y[length//2+ang_window:]

            #x = x[np.array(data.ranges[length[1]//2:]) < 3.0*self.DESIRED_DISTANCE]
            #y = y[np.array(data.ranges[length[1]//2:]) < 3.0*self.DESIRED_DISTANCE]

        
        
        ##VISUALIATION data to consider, green
        #VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser", color=(0,0,1))
        #VisualizationTools.plot_line(front_x, front_y, self.line_pub, frame="/laser", color=(0,1,0))
        #return

        front_wall = np.polyfit(front_x, front_y,1)
        wall = np.polyfit(x,y,1)

        a = wall[0]
        b = wall[1]

        front_a = front_wall[0]
        front_b = front_wall[1]

        #VISUALIATION wall black
        VisualizationTools.plot_line(x, a*x+b, self.line_pub, frame="/laser", color = (0,0,1))
        VisualizationTools.plot_line(front_x, front_a*front_x+front_b, self.front_line_pub, frame="/laser", color = (1,0,1))
        
        theta = np.arctan(a)

        front_theta = np.arctan(front_a)
        rospy.loginfo("Front Wall Angle: %.2f", front_theta)
        fsf = 1 #front steering factor
        
        if self.SIDE == -1:
            error = self.DESIRED_DISTANCE + b*np.cos(theta)
            fsf = 1
        else:
            error = -self.DESIRED_DISTANCE + b*np.cos(theta)
            fsf = -1

        #rospy.loginfo(error)

        P = self.kp #proportion
        I = self.ki #integral
        D = self.kd #derivative


        dt = 1.0/500
        self.integral += error * dt

        derivative = (error - self.previous_error)/dt
        self.previous_error = error
        
        pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = "base_link"
        msg.drive.speed = self.VELOCITY
        msg.drive.acceleration = 0
        msg.drive.steering_angle = P*error + D*derivative + max(min(I*self.integral, 0.34), -0.34)

        rospy.loginfo('dist to front wall:', -front_a/front_b)
        if -front_b/front_a < 2*self.DESIRED_DISTANCE and abs(front_theta) > 1.1:
            msg.drive.steering_angle = 2*fsf*0.34

        msg.drive.steering_angle_velocity = 0

        pub.publish(msg)
        

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
