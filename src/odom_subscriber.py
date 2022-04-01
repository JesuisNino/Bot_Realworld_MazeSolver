#!/usr/bin/env python3
# A simple ROS subscriber node in Python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Subscriber():

    def callback_function(self, odom_data):
        linear_x = odom_data.pose.pose.position.x
        linear_y = odom_data.pose.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion([odom_data.pose.pose.orientation.x, 
                     odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, 
                     odom_data.pose.pose.orientation.w], 
                     'sxyz')

        StartTime = rospy.get_rostime()

      

        if self.wait == 10:
           rospy.loginfo(f"x = {linear_x:.2f}, y = {linear_y:.2f}, yaw = {yaw:.2f}")
           self.wait = 0
        else:
            self.wait += 1

        # if (rospy.get_rostime().secs - StartTime.secs) == 1:
        #     StartTime.secs=rospy.get_rostime().secs
        #     rospy.loginfo(f"x = {linear_x:.2f}, y = {linear_y:.2f}, yaw = {yaw:.2f}")

    def __init__(self):
        self.node_name = "sub_subscriber"
        topic_name = "odom"
        self.wait = 0

        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber(topic_name, Odometry, self.callback_function)
        rospy.loginfo(f"The '{self.node_name}' node is active...")

    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = Subscriber()
    subscriber_instance.main_loop()