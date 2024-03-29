#!/usr/bin/env python3
# Task 1 program to move the robot in the shape of an eight

from os import stat
from this import d
import rospy
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry
# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion
# import some useful mathematical operations (and pi)
from math import sqrt, pow, pi

class MoveEight:
    
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

    def __init__(self):
        node_name = "move_eight"
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() to switch between turning and moving forwards...?
        self.turn = True

        # setup a cmd_vel publisher and an odom subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)

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
    
    def print_stuff(self, a_message):
        # a function to print information to the terminal (use as you wish):
        # print the message that has been passed in to the method via the "a_message" input:
        print(a_message)
        # you could use this to print the current velocity command:
        print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        # you could also print the current odometry to the terminal here, if you wanted to:
        print(f"current odometry: x = {self.x:.3f}, y = {self.y:.3f}, theta_z = {self.theta_z:.3f}")
    
    def main_loop(self):
        status = ""
        diameter = 1
        wait = 0
        
        while not self.ctrl_c:
            
            if self.startup:
                self.vel = Twist()
                status = "init"     
            elif self.turn:
               if abs(self.theta_z) <= 1 and wait > 270: #code to reset the robot for the second loop
                    self.vel = Twist()
                    self.turn = False #Turn Right
                    wait = 0
                    self.x0 = self.x
                    self.y0 = self.y  
                    self.theta_z = self.theta_z0
               else:
                    self.vel = Twist()  #code for the movement of the first loop
                    self.vel.linear.x = 0.12
                    self.vel.angular.z = 0.24 / diameter
                    wait += 1
                    status = "running left"
            else:
                if abs(self.theta_z) >= 0.015 and wait < 268:  #code for the movement of the second loop
                    self.vel = Twist()
                    self.vel.linear.x = 0.12
                    self.vel.angular.z = - 0.24 / diameter
                    wait += 1
                    status = "running right"    
                else:
                    self.vel = Twist()
                    

            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

if __name__ == '__main__':
    moveeight_instance = MoveEight()
    try:
        moveeight_instance.main_loop()
    except rospy.ROSInterruptException:
        pass