#! /usr/bin/env python3
from math import sqrt, pow
import numpy as np
# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import the tb3 modules (which needs to exist within the "week6_vision" package)

from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal

from tb4 import Tb3Move, Tb3Odometry, Tb3LaserScan

import actionlib

class SearchActionServer(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()
        self.move=1

        self.one=1

        self.first=1

        self.second=1

        self.three=1

        self.m00 = 0
        self.m00_min = 200000
        self.m00_mim1 = 600000

        # Thresholds for ["red", "yellow", "Green", "Turquoise","blue","puerple"]
        self.lower = [(-1.4, 205, 100), (28, 194, 100), (58, 228, 100), (89, 201, 100),(118,235,100),(149,230,100)]
        self.upper = [(1.3, 255, 255), (32, 255, 255), (60, 255, 255), (91, 255, 255),(132,256,255),(151,260,255)]
    
    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    
    
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

        
        mask_red = cv2.inRange(hsv_img, self.lower[0], self.upper[0])      
        mask_yellow = cv2.inRange(hsv_img, self.lower[1], self.upper[1])
        mask_green = cv2.inRange(hsv_img, self.lower[2], self.upper[2])
        mask_turquoise = cv2.inRange(hsv_img, self.lower[3], self.upper[3])
        mask_blue = cv2.inRange(hsv_img, self.lower[4], self.upper[4])
        mask_puerple = cv2.inRange(hsv_img, self.lower[5], self.upper[5])



        m_red = cv2.moments(mask_red)
        m_yellow = cv2.moments(mask_yellow)
        m_green = cv2.moments(mask_green)
        m_turquoise = cv2.moments(mask_turquoise)
        m_blue = cv2.moments(mask_blue)
        m_puerple = cv2.moments(mask_puerple)



        self.m00_red = m_red["m00"]
        self.cy = m_red["m10"] / (m_red["m00"] + 1e-5)

        self.m00_yellow = m_yellow["m00"]
        self.cy = m_yellow["m10"] / (m_yellow["m00"] + 1e-5)

        self.m00_green = m_green["m00"]
        self.cy = m_green["m10"] / (m_green["m00"] + 1e-5)

        self.m00_turquoise = m_turquoise["m00"]
        self.cy = m_turquoise["m10"] / (m_turquoise["m00"] + 1e-5)

        self.m00_blue = m_blue["m00"]
        self.cy = m_blue["m10"] / (m_blue["m00"] + 1e-5)

        self.m00_puerple = m_puerple["m00"]
        self.cy = m_puerple["m10"] / (m_puerple["m00"] + 1e-5)


        if self.m00_red > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        if self.m00_yellow > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        if self.m00_green > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        if self.m00_turquoise > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        if self.m00_blue > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        if self.m00_puerple > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow("cropped image", crop_img)
        cv2.waitKey(1)


    def action_server_launcher(self, goal: SearchGoal):
        r = rospy.Rate(10)

        success = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
            print("Invalid velocity.  Select a value between 0 and 0.26 m/s.")
            success = False
        if goal.approach_distance <= 0.2:
            print("Invalid approach distance: I'll crash!")
            success = False
        elif goal.approach_distance > 3.5:
            print("Invalid approach distance: I can't measure that far.")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print(f"Request to move at {goal.fwd_velocity:.3f}m/s "
                f"and stop {goal.approach_distance:.2f}m "
                f"infront of any obstacles")

        # Get the current robot odometry:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy

        print("The robot will start to move now...")
        # set the robot velocity:
        

        if self.first==1:
           while self.three==1:

            while self.tb3_lidar.min_distance>0.5:
                self.vel_controller.set_move_cmd(0, 0.25)
                self.vel_controller.publish()
                
            else:
                if self.m00_turquoise > self.m00_min:
                    print("Detected Turquoise")
                    while self.second==1:
                      if self.one==1:
                        if self.tb3_lidar.min_right>0.31:
                            self.vel_controller.set_move_cmd(0, -0.25)
                            self.vel_controller.publish()
                    
                        else:
                            self.vel_controller.set_move_cmd(0, 0.0)
                            self.vel_controller.publish()
                            self.one=0
                            
                      else:
                        
                        while self.move==1:
                            if self.m00_turquoise > self.m00_min:
                
                                if self.m00_turquoise>200000:
                                    if self.tb3_lidar.min_distance>0.35:

                                        self.vel_controller.set_move_cmd(0.26, 0)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, 0)
                                        self.vel_controller.publish()
                                        print("The target color is detected and the end point is reached.")

                
                                else:          
                                    if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.27 and self.tb3_lidar.min_right>0.27:
                                        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                        self.vel_controller.publish()
                                    elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                        self.vel_controller.set_move_cmd(0, 1.5)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, -1.5)
                                        self.vel_controller.publish()

            
                            else:
                                if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.26 and self.tb3_lidar.min_right>0.26:
                                    self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                    self.vel_controller.publish()
                                elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                    self.vel_controller.set_move_cmd(0, 1.5)
                                    self.vel_controller.publish()
                                else:
                                    self.vel_controller.set_move_cmd(0, -1.5)
                                    self.vel_controller.publish()
                
#################################################################################################
                if self.m00_green > self.m00_min:
                    print("Detected Green")
                    while self.second==1:
                      if self.one==1:
                        if self.tb3_lidar.min_right>0.31:
                            self.vel_controller.set_move_cmd(0, -0.2)
                            self.vel_controller.publish()
                    
                        else:
                            self.vel_controller.set_move_cmd(0, 0.0)
                            self.vel_controller.publish()
                            self.one=0
                            

                      else:
                        
                        while self.move==1:
                            if self.m00_green > self.m00_min:
                
                                if self.m00_green>20000:
                                    if self.tb3_lidar.min_distance>0.45:

                                        self.vel_controller.set_move_cmd(0.26, 0)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, 0)
                                        self.vel_controller.publish()
                                        print("The target color is detected and the end point is reached.")

                
                                else:          
                                    if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.27 and self.tb3_lidar.min_right>0.27:
                                        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                        self.vel_controller.publish()
                                    elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                        self.vel_controller.set_move_cmd(0, 1.5)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, -1.5)
                                        self.vel_controller.publish()

            
                            else:
                                if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.27 and self.tb3_lidar.min_right>0.27:
                                    self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                    self.vel_controller.publish()


                                elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                    self.vel_controller.set_move_cmd(0, 1.5)
                                    self.vel_controller.publish()
                                else:
                                    self.vel_controller.set_move_cmd(0, -1.5)
                                    self.vel_controller.publish()                    

                        
                
                

#################################################################################################
                if self.m00_blue > self.m00_min:
                    print("Detected Blue")
                    while self.second==1:
                      if self.one==1:
                        if self.tb3_lidar.min_right>0.31:
                            self.vel_controller.set_move_cmd(0, -0.2)
                            self.vel_controller.publish()
                    
                        else:
                            self.vel_controller.set_move_cmd(0, 0.0)
                            self.vel_controller.publish()
                            self.one=0
                            

                      else:
                        
                        while self.move==1:
                            if self.m00_blue > self.m00_min:
                
                                if self.m00_blue>20000:
                                    if self.tb3_lidar.min_distance>0.45:

                                        self.vel_controller.set_move_cmd(0.26, 0)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, 0)
                                        self.vel_controller.publish()
                                        print("The target color is detected and the end point is reached.")

                
                                else:          
                                    if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.27 and self.tb3_lidar.min_right>0.27:
                                        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                        self.vel_controller.publish()
                                    elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                        self.vel_controller.set_move_cmd(0, 1.5)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, -1.5)
                                        self.vel_controller.publish()

            
                            else:
                                if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.28 and self.tb3_lidar.min_right>0.28:
                                    self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                    self.vel_controller.publish()
                                elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                    self.vel_controller.set_move_cmd(0, 1.45)
                                    self.vel_controller.publish()
                                else:
                                    self.vel_controller.set_move_cmd(0, -1.45)
                                    self.vel_controller.publish()                    


#################################################################################################
                if self.m00_yellow > self.m00_min:
                    print("Detected Yellow")
                    while self.second==1:
                      if self.one==1:
                        if self.tb3_lidar.min_right>0.31:
                            self.vel_controller.set_move_cmd(0, -0.2)
                            self.vel_controller.publish()
                    
                        else:
                            self.vel_controller.set_move_cmd(0, 0.0)
                            self.vel_controller.publish()
                            self.one=0
                            

                      else:
                        
                        while self.move==1:
                            if self.m00_yellow > self.m00_min:
                
                                if self.m00_yellow>20000:
                                    if self.tb3_lidar.min_distance>0.45:

                                        self.vel_controller.set_move_cmd(0.26, 0)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, 0)
                                        self.vel_controller.publish()
                                        print("The target color is detected and the end point is reached.")

                
                                else:          
                                    if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.27 and self.tb3_lidar.min_right>0.27:
                                        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                        self.vel_controller.publish()
                                    elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                        self.vel_controller.set_move_cmd(0, 1.5)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, -1.5)
                                        self.vel_controller.publish()

            
                            else:
                                if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.27 and self.tb3_lidar.min_right>0.27:
                                    self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                    self.vel_controller.publish()
                                elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                    self.vel_controller.set_move_cmd(0, 1.5)
                                    self.vel_controller.publish()
                                else:
                                    self.vel_controller.set_move_cmd(0, -1.5)
                                    self.vel_controller.publish()      


#################################################################################################
                if self.m00_red > self.m00_min:
                    print("Detected Red")
                    while self.second==1:
                      if self.one==1:
                        if self.tb3_lidar.min_right>0.31:
                            self.vel_controller.set_move_cmd(0, -0.2)
                            self.vel_controller.publish()
                    
                        else:
                            self.vel_controller.set_move_cmd(0, 0.0)
                            self.vel_controller.publish()
                            self.one=0
                            

                      else:
                        
                        while self.move==1:
                            if self.m00_red > self.m00_min:
                
                                if self.m00_red>20000:
                                    if self.tb3_lidar.min_distance>0.45:

                                        self.vel_controller.set_move_cmd(0.26, 0)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, 0)
                                        self.vel_controller.publish()
                                        print("The target color is detected and the end point is reached.")

                
                                else:          
                                    if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.27 and self.tb3_lidar.min_right>0.27:
                                        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                        self.vel_controller.publish()
                                    elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                        self.vel_controller.set_move_cmd(0, 1.5)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, -1.5)
                                        self.vel_controller.publish()

            
                            else:
                                if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.27 and self.tb3_lidar.min_right>0.27:
                                    self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                    self.vel_controller.publish()
                                elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                    self.vel_controller.set_move_cmd(0, 1.5)
                                    self.vel_controller.publish()
                                else:
                                    self.vel_controller.set_move_cmd(0, -1.5)
                                    self.vel_controller.publish()                                           
            
#################################################################################################
                if self.m00_puerple > self.m00_min:
                    print("Detected Puerple")
                    while self.second==1:
                      if self.one==1:
                        if self.tb3_lidar.min_right>0.31:
                            self.vel_controller.set_move_cmd(0, -0.2)
                            self.vel_controller.publish()
                    
                        else:
                            self.vel_controller.set_move_cmd(0, 0.0)
                            self.vel_controller.publish()
                            self.one=0
                           

                      else:
                        
                        while self.move==1:
                            if self.m00_puerple > self.m00_min:
                
                                if self.m00_puerple>20000:
                                    if self.tb3_lidar.min_distance>0.45:

                                        self.vel_controller.set_move_cmd(0.26, 0)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, 0)
                                        self.vel_controller.publish()
                                        print("The target color is detected and the end point is reached.")

                
                                else:          
                                    if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.27 and self.tb3_lidar.min_right>0.27:
                                        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                        self.vel_controller.publish()
                                    elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                        self.vel_controller.set_move_cmd(0, 1.5)
                                        self.vel_controller.publish()
                                    else:
                                        self.vel_controller.set_move_cmd(0, -1.5)
                                        self.vel_controller.publish()

            
                            else:
                                if self.tb3_lidar.min_distance > 0.55 and self.tb3_lidar.min_left>0.27 and self.tb3_lidar.min_right>0.27:
                                    self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                                    self.vel_controller.publish()
                                elif self.tb3_lidar.min_left>self.tb3_lidar.min_right:
                                    self.vel_controller.set_move_cmd(0, 1.5)
                                    self.vel_controller.publish()
                                else:
                                    self.vel_controller.set_move_cmd(0, -1.5)
                                    self.vel_controller.publish()   


            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))
            # populate the feedback message and publish it:
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)

        if success:
            rospy.loginfo("approach completed sucessfully.")
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.tb3_lidar.min_distance
            self.result.closest_object_angle = self.tb3_lidar.closest_object_position

            self.actionserver.set_succeeded(self.result)
            self.vel_controller.stop()
            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
