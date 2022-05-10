#!/usr/bin/env python3
# Task 1 program to move the robot in the shape of an eight

from os import stat
from this import d
import rospy
import numpy as np
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry
# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion
# import some useful mathematical operations (and pi)
from math import sqrt, pow, pi
from tb3 import Tb3LaserScan
class MoveMaze:
    
    def callback_function(self, odom_data):
        # obtain the orientation co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        # If this is the first time that this callback_function has run, then 
        # obtain a "reference position" (used to determine how far the robot has moved
        # during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z


    def scan_callback(self, scan_data):
     
        # get range of front orientation(between 5 to -5 degrees)
        range_front = scan_data.ranges[-5:5]
        front_arc = np.array(range_front[::-1])
        # get range of left orientation(between 15 to 60 degrees)
        range_left = scan_data.ranges[15:60]
        left_arc = np.array(range_left[::-1])
        # get the minimum values of each orientation
        self.min_front = front_arc.min()
        self.min_left = left_arc.min()

    def __init__(self):
        node_name = "move_eight"
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() to switch between turning and moving forwards...?
        self.turn = True

        # setup a cmd_vel publisher and an odom subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)

        self.tb3_lidar = Tb3LaserScan()

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        # define the robot pose variables and set them all to zero to start with:
        # variables to use for the "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.near_wall = 0
        
        # define a Twist instance, which can be used to set robot velocities
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True
    
    
    
    def main_loop(self):
        

        while not self.ctrl_c:
            if self.startup:
                self.vel = Twist()
             #Make the robot move forward until the distance from the front is 0.32
            elif self.near_wall == 0: 
                self.vel=Twist()
                if self.tb3_lidar.min_distance > 0.32:
                    self.vel.angular.z = 0.0  
                    self.vel.linear.x = 0.26

                else:
                    self.vel.angular.z = 0.0
                    self.vel.linear.x = 0.0
                    self.near_wall = 1
                
            
            else:
                 #When the distance from the front is greater than 0.4
                if self.tb3_lidar.min_distance > 0.4: 
                     #When too close to the wall, turn slightly to the right
                    if self.tb3_lidar.min_left<0.28:
                        self.vel.angular.z = -0.6
                        self.vel.linear.x = 0.26
                     #Go straight in this area
                    elif 0.28<self.tb3_lidar.min_left<0.33:
                        self.vel.angular.z = 0.0
                        self.vel.linear.x = 0.26
                     #There is no obstacle on the left, turn left
                    else:
                        self.vel.angular.z = 1.0
                        self.vel.linear.x = 0.26
                 #Obstacles ahead and left, turn right
                else:
                    self.vel.angular.z = -1.2
                    self.vel.linear.x = 0.0

            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

if __name__ == '__main__':
    movemaze_instance = MoveMaze()
    try:
        movemaze_instance.main_loop()
    except rospy.ROSInterruptException:
        pass