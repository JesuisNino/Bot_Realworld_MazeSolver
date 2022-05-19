#!/usr/bin/env python3

import numpy as np
import cv2

import rospkg
import roslaunch

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid


BOX_colourS = {
    "red": {"colour_lower": np.array([0, 185, 100]), "colour_upper": np.array([10, 255, 255])},
    "yellow": {"colour_lower": np.array([26, 43, 46]), "colour_upper": np.array([34, 255, 255])},
    "green": {"colour_lower": np.array([25, 150, 100]), "colour_upper": np.array([70, 255, 255])},   
    "blue": {"colour_lower": np.array([115, 224, 100]), "colour_upper": np.array([130, 255, 255])}
}

## process rgbd image by cv to find the target object
class image_converter:

  def __init__(self):
    self.target_colour = rospy.get_param("/target_colour", "blue")
    print("target_colour:", self.target_colour)
    #Subscriber image message   
    self.target_pic_area = 0   
    rospack = rospkg.RosPack()
    # print(rospack.list())
    self.pic_path = rospack.get_path('team47')+"/snaps/the_beacon.jpg" 
    print(self.pic_path)             
    self.image_sub = rospy.Subscriber( "/camera/rgb/image_raw", Image, self.callback)
    #Publisher BoundingBoxes message for objection
    self.bridge = CvBridge()
    
    self.map = OccupancyGrid()
    self.last_update_map_time = rospy.Time(0)  
    self.map_path = rospack.get_path('team47')+"/maps/task5_map"
    print("Saving map   map_path:", self.map_path)
    self.enable_save_map = False
    self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
    self.LoopRun()

  def LoopRun(self):
      #Main Loop.
      rospy.loginfo("LoopRun:")
      while not rospy.is_shutdown():
        if self.enable_save_map :
          self.enable_save_map = False
          args_str = " -f " + self.map_path      
          node = roslaunch.core.Node(package="map_server", node_type="map_saver", args=args_str)
          launch = roslaunch.scriptapi.ROSLaunch()
          launch.start()                                  
          process = launch.launch(node)
          print("Saved the map:", self.map_path)
        rospy.sleep(0.1)

  def map_callback(self, data):
      # Callback function for map subscriber.
      # Subscribes to /map to get the OccupancyGrid of the map.

      cur_time = rospy.Time.now()
      time_diff = (cur_time - self.last_update_map_time).to_sec()
      if time_diff > 30.0 :
        self.last_update_map_time = cur_time
        self.enable_save_map = True
        print("update the OccupancyGrid map!")


  def cv_show(self, name, img):
      cv2.imshow(name, img)
      cv2.waitKey(3)

  def callback(self, data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # process image by GaussianBlur filter
    frame_ = cv2.GaussianBlur(frame, (5, 5), 0)
    # convert the colour style of image
    hsv = cv2.cvtColor(frame_, cv2.COLOR_BGR2HSV)

    # find the box with the candidate colour within BOX_colourS
    colour = self.target_colour
    # image filter
    mask = cv2.inRange( hsv, BOX_colourS[colour]["colour_lower"], BOX_colourS[colour]["colour_upper"])
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    mask = cv2.GaussianBlur(mask, (3, 3), 0)
    
    # find contours about obj
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    has_found = False
    for cnt in cnts:
        # rectange info, point(x,y) width and heigh
        x, y, w, h = cv2.boundingRect(cnt)
        rect = cv2.minAreaRect(cnt)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        cur_area = (int)(w*h)
        if cur_area > (int)(self.target_pic_area*1.05):
          self.target_pic_area = cur_area
          has_found = True
          print("cur_area:", cur_area)

    if has_found :
      self.save_image(frame)
    #self.cv_show("img", frame)

  def save_image(self, img):
      print("Saved an image to :", self.pic_path)
      cv2.imwrite(str(self.pic_path), img)

if __name__ == '__main__':
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  print('start')

  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    print('exception')
