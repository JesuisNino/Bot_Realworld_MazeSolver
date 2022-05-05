#!/usr/bin/env python
import imp
import rospy # Python library for ROS

from maze_controller import Controller
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #

def callback(tb):
    tb = Controller()
    turn = False

    thr1 = 0.8 # Laser scan range threshold
    thr2 = 0.8
    if tb.ranges[0]>thr1 and tb.ranges[15]>thr2 and tb.ranges[345]>thr2: # Checks if there are obstacles in front and
                                                                         # 15 degrees left and right (Try changing the
									 # the angle values as well as the thresholds)
        tb.moveForward(0.5)
        turn = False
    else:
        tb.turnLeft(90)
        turn = True
        if tb.ranges[0]>thr1 and tb.ranges[15]>thr2 and tb.ranges[345]>thr2:
            tb.moveForward(0.5)
            turn = False
    pub.publish(tb) # publish the object
    # Get the position of the robot from the onboard sensors

vel = Twist() # Creates a Twist message type object
rospy.init_node('maze_nav') # Initializes a node
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                            				 # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                         # outgoing message queue used for asynchronous publishing

sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

rospy.spin() # Loops infinitely until someone stops the program execution
