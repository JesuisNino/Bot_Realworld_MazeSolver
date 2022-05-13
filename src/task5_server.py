#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose, Point, Quaternion
from random import randrange
import time

class Explore:

    def __init__(self):
        # Initialize rate:
        self.rate = rospy.Rate(1)
        # Simple Action Client:
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.logdebug("move_base is ready") 

        self.x = 0
        self.y = 0
        self.completion = 0

        # Initialize subscribers:
        self.map = OccupancyGrid()
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.loc_path_sub = rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.local_path_callback)
        self.local_path_callback = rospy.Time.now()
        time.sleep(8)

    def local_path_callback(self, data):
        self.last_loc_path_time = rospy.Time.now()

    def map_callback(self, data):
        #Callback function for map subscriber.
        #Subscribes to /map to get the OccupancyGrid of the map.
        valid = False
        self.map = data
        map_size = 0
        while valid is False:
            map_size = randrange(len(data.data))
            map_cell_value = data.data[map_size]

            edges = self.check_neighbors(data, map_size)
            if map_cell_value != -1 and map_cell_value <= 0.20 and edges is True:
                valid = True

        if map_size == 0:
          return 
          
        row = map_size / self.map.info.width
        col = map_size % self.map.info.width

        self.x = col * self.map.info.resolution + self.map.info.origin.position.x  # column * resolution + origin_x
        self.y = row * self.map.info.resolution + self.map.info.origin.position.y  # row * resolution + origin_x
        # print(row, col, self.x, self.y)

        if self.completion % 2 == 0 or (rospy.Time.now() - self.last_loc_path_time).to_sec() > 5.0:
            self.last_loc_path_time = rospy.Time.now()
            self.completion += 1
            # Start the robot moving toward the goal
            self.set_goal()
            print("set_goal:", self.x, self.y, "loc_time", (rospy.Time.now() - self.last_loc_path_time).to_sec())
    

    def set_goal(self):
        #Set goal position for move_base.
        rospy.logdebug("Setting goal")

        # Create goal:
        goal = MoveBaseGoal()

        # Set random goal:
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.orientation.w = 1.0
        rospy.logdebug(f"goal: {goal.target_pose.pose.position.x, goal.target_pose.pose.position.y}")
        self.move_base.send_goal(goal, self.goal_status)


    def goal_status(self, status, result):
        # Check the status of a goal - goal reached, aborted, or rejected.

        self.completion += 1

        # Goal reached
        if status == 3:
            rospy.loginfo("Goal succeeded")

        # Goal aborted
        if status == 4:
            rospy.loginfo("Goal aborted")

        # Goal rejected
        if status == 5:
            rospy.loginfo("Goal rejected")


    def check_neighbors(self, data, map_size):
        # Checks neighbors for random points on the map.
        unknowns = 0
        obstacles = 0

        for x in range(-3, 4):
            for y in range(-3, 4):
                row = x * self.map.info.width + y
                try:
                    if data.data[map_size + row] == -1:
                        unknowns += 1
                    elif data.data[map_size + row] > 0.65:
                        obstacles += 1
                except IndexError:
                    pass
        if unknowns > 0 and obstacles < 2:
            return True
        else:
            return False


def main():
    rospy.init_node('explore', log_level=rospy.DEBUG)
    Explore()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException