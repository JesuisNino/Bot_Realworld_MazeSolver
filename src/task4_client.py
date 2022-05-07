#! /usr/bin/env python3

import rospy
import actionlib

from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow, pi
import numpy as np

class action_client(object):
    STEPS = [2, 1.3, 0.8, 0.4]
   
    def feedback_callback(self, feedback_data: SearchFeedback):
        self.distance = feedback_data.current_distance_travelled
        if self.i < 100:
            self.i += 1
        else:
            self.i = 0
            # print(f"FEEDBACK: Currently travelled {self.distance:.3f} m")

        
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # create a single mask to accommodate all four dectection colours:
        for i in range(4):
            if i == 0:
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
            else:
                mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

        m = cv2.moments(mask)
            
        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow("cropped image", crop_img)
        cv2.waitKey(1)

    def turn_left(self, degree):
        turn_velocity = 1.3
        angle = degree * pi / 180
        print("Turning left at angle: " + str(degree) + " degrees")
            
        self.vel_controller.set_move_cmd(0.0, turn_velocity)
        turn_time = abs(angle / turn_velocity)

        print("Turning for " + str(turn_time) +" seconds at " + str(turn_velocity) +" m/s")
        loop_initial_time = rospy.get_time()
        while rospy.get_time() < loop_initial_time + turn_time:
            self.vel_controller.publish()
        self.vel_controller.stop()

    def turn_right(self, degree):
        turn_velocity = -1.3
        angle = - (degree * pi / 180)
        print("Turning right at angle: " + str(degree) + " degrees")
            
        self.vel_controller.set_move_cmd(0.0, turn_velocity)
        turn_time = abs(angle / turn_velocity)

        print("Turning for " + str(turn_time) +" seconds at " + str(turn_velocity) +" m/s")
        loop_initial_time = rospy.get_time()
        while rospy.get_time() < loop_initial_time + turn_time:
            self.vel_controller.publish()
        self.vel_controller.stop()    
    def __init__(self):
        self.action_complete = False
        rospy.init_node("obstacle_action_client")
        self.rate = rospy.Rate(1)
        self.goal = SearchGoal()

        self.vel_controller = Tb3Move()

        self.client = actionlib.SimpleActionClient("/obstacle_action_server", 
                    SearchAction)
                    
        self.client.wait_for_server()
        self.distance = 0.0
        self.i = 0
        self.step = 0
        ###########################
        node_name = "turn_and_face"
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 100000

        # Thresholds for ["Blue", "Red", "Green", "Turquoise"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255)]
    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")
        self.ctrl_c = True
            
    def send_goal(self, velocity, approach):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = approach
        
        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        while not self.ctrl_c:
            self.send_goal(velocity = 0.2, approach = 0.5)
            prempt = False
            while self.client.get_state() < 2:
                if self.distance >= self.STEPS[self.step]:
                    rospy.logwarn("Traversed "+ str(self.STEPS[self.step]) + " metres, turning randomly")
                    self.client.cancel_goal()
                    random_degree = np.random.uniform(0,360)
                    self.turn_left(random_degree)
                    prempt = True
                    break
                if self.distance >= self.STEPS[self.step] and self.m00 > self.m00_min:
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == "slow":
                         self.move_rate = "stop"
                         self.stop_counter = 20
                    else:
                        self.move_rate = "slow"
                else:
                    self.move_rate = "fast"
                
            if self.move_rate == "fast":
                print(f"MOVING FAST: I can't see anything at the moment (blob size = {self.m00:.0f}), scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == "slow":
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == "stop" and self.stop_counter > 0:
                print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
                self.robot_controller.set_move_cmd(0.0, 0.0)
            else:
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.robot_controller.publish()

            self.rate.sleep()
            
            self.action_complete = True
            if prempt:
                self.step += 1
                if self.step == 4:
                    self.step = 0
            else:
                result = self.client.get_result()
            self.vel_controller.stop()   

if __name__ == '__main__':
    client_instance = action_client()
    try:
        client_instance.main()
    except rospy.ROSInterruptException:
        pass