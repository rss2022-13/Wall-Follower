#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class SafetyController:
    
    
    def __init__(self):

        #SUBSCRIBER
        self.driver = rospy.Subscriber("/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped, self.driver_data)
        self.scan = rospy.Subscriber("/scan", LaserScan, self.scan_data)
        self.pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=1)
        self.speed = 0
        self.acceleration = 0
        self.steering_angle = 0
        self.steering_angle_velocity = 0
        self.ranges = []
        self.angle_min = -2.35
        self.angle_max = 2.35
        self.increment = 0
        self.max_speed = 4
        self.parabola_const = -1
        self.parabola_offset = 1
        self.desired_width = 2
        #self.breaking_distance = 0.1
        
    def driver_data(self, data):
        self.speed = data.drive.speed 
        self.steering_angle = data.drive.acceleration
        self.steering_angle = data.drive.steering_angle 
        self.steering_angle_velocity = data.drive.steering_angle_velocity

        self.parabola_offset = -.3 * self.speed
        self.parabola_const = -self.parabola_offset/self.desired_width**2        
        
    def scan_data(self, data):
        self.ranges = data.ranges
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.increment = data.angle_increment
        self.publish(data)
        
    def get_wall(self,x, y):
        wall = np.polyfit(x,y,1)

        a = wall[0]
        b = wall[1]
        return a, b
    
    def publish(self,data):
        #define parabola
        theta_total = np.linspace(data.angle_min, data.angle_max, len(data.ranges))
        #parabola is a np array of radial distances that range from 0->pi radians that map a parabola
        # parabola = self.parabola_polar(theta)
        lidar_x = np.array(data.ranges * np.cos(theta_total))
        lidar_y = np.array(data.ranges * np.sin(theta_total))
        
        phi = self.steering_angle
        #cos = np.cos(phi)
        #rot = np.array([[ cos, -np.sin(phi)], [np.sin(phi), cos]])

        #gets parabola in polar form
        theta_parabola = np.linspace(0, np.pi, np.pi/data.angle_increment)

        r = np.array(list(map(self.parabola_polar, theta_parabola)))

        #TODO: discard data too far away - maybe necessary
        
        #get 3 walls
        size = len(data.ranges)
        a1, b1 = self.get_wall(lidar_x[:int(size/3)], lidar_y[:int(size/3)])
        front_rotated = np.vstack((lidar_x[int(size/3): int(2*size/3)], lidar_y[int(size/3): int(2*size/3)]))
        #rot90 = np.matrix([[np.cos(np.pi/2), -np.sin(np.pi/2)], [np.sin(np.pi/2), np.cos(np.pi/2)]])
        #front_rotated = np.matmul(rot90, front_wall)
        #rospy.loginfo(front_rotated[0,:].flatten().shape)
        a2, b2 = self.get_wall(front_rotated[0,:].flatten(), front_rotated[1,:].flatten())
        a3, b3 = self.get_wall(lidar_x[int(2*size/3):], lidar_y[int(2*size/3):])
        
        start_angle = self.steering_angle - np.pi/2
        end_angle = self.steering_angle + np.pi/2
        active_range = np.linspace(start_angle,end_angle,np.pi/data.angle_increment)
        
        #TODO: convert wall data to polar form
        left = np.sqrt(lidar_x[:int(size/3)]**2 + (a1*lidar_x[:int(size/3)] + b1)**2)
        center = np.sqrt(lidar_x[int(size/3): int(2*size/3)]**2 + (a2*lidar_x[int(size/3): int(2*size/3)] + b2)**2)
        right = np.sqrt(lidar_x[int(2*size/3):]**2 + (a3*lidar_x[int(2*size/3):] + b3)**2)

        #if we concatenate all walls, we can now index into them quickly with a given range of thetas
        all_walls = np.hstack((right,center,left))[int(start_angle*data.angle_increment):int(end_angle*data.angle_increment)]

        #compare wall and bubble location to see if we need to apply safety control
        apply_safety = False
        collision_wall = None
        for i in range(len(all_walls)):
            if all_walls[i] < r[i]:
                apply_safety = True
                if int(start_angle/data.angle_increment) + i in range(len(theta_total/3)):
                    collision_wall = 1
                elif int(start_angle/data.angle_increment) + i in range(len(theta_total)/3,2*len(theta_total)/3):
                    collision_wall = 0
                elif int(start_angle/data.angle_increment) + i in range(2*len(theta_total)/3,len(theta_total)):
                    collision_wall = -1
                break
        
        # if True in np.array(r[:int(size/3.0)] - left >= 0):
        #     apply_safety = True
        #     collision_wall = -1
        # if True in np.array(r[int(size/3.0): int(2*size/3.0)] - center >= 0):
        #     apply_safety = True
        #     collision_wall = 0
        # if True in np.array(r[int(2*size/3.0):] - right >= 0):
        #     apply_safety = True
        #     collision_wall = 1
        
        #TODO: find and publish acceleration value
        
        #if obstacle within x distance in direction robot travelling,
        #slow down car at rate proportional to initial speed

        #v_f^2 = 0 = v_0^2 + 2ax;  x = breaking distance, publish a
        #breaking distance depends on:
        # current velocity
        # distance to crash
        # angle between direction travelled and obstacle
        rospy.loginfo("hello")
        if apply_safety:
            #relative angle between wall and steering angle
            delta = None
            distance_to_wall = None
            if collision_wall == -1:
                delta = np.arctan(a1)
                distance_to_wall = b1
            elif collision_wall == 1:
                delta = np.arctan(a3)
                distance_to_wall = b3
            else:
                delta = np.arctan(a2)
                distance_to_wall = b2
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time(0)
            msg.header.frame_id = "base_link"
            #msg.drive.speed = 
            msg.drive.acceleration = -(self.speed*np.sin(delta))**2/(2*distance_to_wall)
            #msg.drive.steering_angle = 
            #msg.drive.steering_angle_velocity = 0
    
            self.pub.publish(msg)


    def parabola_polar(self,theta):
        '''
        Returns the r value which would allow a polar ray to intersect with a cartesian parabola at a certain theta value.
        Is very helpful for getting a polar representation of a parabola with set thetas.

        This function acts on a single theta value, not an array of thetas because of the edge case at pi/2 where the distance is undefined
        '''
        if theta == np.pi/2:
            return self.parabola_offset

        r1 = (-np.sin(theta) + np.sqrt(np.sin(theta)**2-4*self.parabola_const*self.parabola_offset*np.cos(theta)**2))/(-2*self.parabola_const*np.cos(theta)**2)
        # r2 = (-np.sin(theta) - np.sqrt(np.sin(theta)**2-4*self.parabola_const*self.parabola_offset*np.cos(theta)**2))/(-2*self.parabola_const*np.cos(theta)**2)

        #return radial distances for a list of thetas
        return r1
        

    def parabola_y(self,x):
        '''
        in order to preserve the structure of our parabola, so that we don't get too small and clip the edges of the vehicle,
        we need to make sure that when x=1/2width of car, |parabola_const*x^2| >= |parabola_offset|
        '''
        return self.parabola_const*x**2+self.parabola_offset

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    safety_controller = SafetyController()
    # safety_controller.publish()
    rospy.spin()
