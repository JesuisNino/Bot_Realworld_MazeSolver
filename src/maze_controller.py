import rospy
from math import radians, degrees, sqrt
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class Controller():

    def __init__(self):
        self.x = 0
        self.y = 0
        self.angle = 0
        self.LIDAR_ERR = 0.05
        self.rate = rospy.Rate(10)

        # Register publishers and subscribers
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odomCallback)

        rospy.on_shutdown(self.shutdown)

    # Get the position of the robot from the onboard sensors
    def odomCallback(self, odom):
        roll, pitch, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        self.angle = (round(degrees(yaw)) + 180) % 360
        self.x = round(odom.pose.pose.position.x, 3)
        self.y = round(odom.pose.pose.position.y, 3)

    # Move the bot forward at speed desired
    def moveForward(self, speed):
        move_cmd = Twist()
        move_cmd.linear.x = speed
        self.publisher.publish(move_cmd)

    # Turn to the left of a specified angle (degrees)
    def turnLeft(self, angle):
        move_cmd = Twist()
        move_cmd.angular.z = radians(angle / 4)
        start_angle = self.angle
        end_angle = (start_angle + angle) % 360
        if self.angle < end_angle:
        	while end_angle > self.angle and not rospy.is_shutdown():
                    self.publisher.publish(move_cmd)
                    self.rate.sleep()
        elif self.angle > end_angle:
        	while self.angle > 0 and self.angle > end_angle and not rospy.is_shutdown():
                    self.publisher.publish(move_cmd)
                    self.rate.sleep()
        move_cmd = Twist()
        self.publisher.publish(move_cmd)

    # Turn to the right of a specified angle (degrees)
    def turnRight(self, angle):
        move_cmd = Twist()
        move_cmd.angular.z = -radians(angle / 4)
        start_angle = self.angle
        end_angle = ((start_angle - angle) + 360) % 360
        if self.angle > end_angle:
        	while end_angle < self.angle and not rospy.is_shutdown():
                    self.publisher.publish(move_cmd)
                    self.rate.sleep()
        elif self.angle < end_angle:
        	while self.angle > 0 and self.angle < end_angle and not rospy.is_shutdown():
                    self.publisher.publish(move_cmd)
                    self.rate.sleep()
        move_cmd = Twist()
        self.publisher.publish(move_cmd)

    # Detect a hole in the maze to the right
    def detectHoleRight(self):
        msg = rospy.wait_for_message("/scan", LaserScan)
        self.scan_filter = []
        for i in range(360):
            if i >= 255 and i < 260:
                if msg.ranges[i] >= self.LIDAR_ERR:
                    self.scan_filter.append(msg.ranges[i])
        if len(self.scan_filter) == 0:
            return False
        else:
            return max(self.scan_filter) >= 0.6

    # Detect a hole in the maze to the left
    def detectHoleLeft(self):
        msg = rospy.wait_for_message("/scan", LaserScan)
        self.scan_filter = []
        for i in range(360):
            if i >= 100 and i < 105:
                if msg.ranges[i] >= self.LIDAR_ERR:
                    self.scan_filter.append(msg.ranges[i])
        if len(self.scan_filter) == 0:
            return False
        else:
            return max(self.scan_filter) >= 0.6

    # Stop the bot and shutdown the program
    def shutdown(self):
        self.stop()
        rospy.loginfo("Stopping Robot")
        exit(0)

    # Stop the bot
    def stop(self):
        move_cmd = Twist()
        self.publisher.publish(move_cmd)