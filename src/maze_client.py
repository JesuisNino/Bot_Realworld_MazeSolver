#! /usr/bin/env python3

import rospy
import actionlib

from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import degrees, sqrt, pow, pi
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
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)
        self.distance = 0.0
        self.i = 0
        self.step = 0

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
            
            

            self.vel_controller.stop()   

if __name__ == '__main__':
    client_instance = action_client()
    try:
        client_instance.main()
    except rospy.ROSInterruptException:
        pass