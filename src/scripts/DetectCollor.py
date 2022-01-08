#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",7Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback) #for Gazebo tourtlebot
    #self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback) #for real robot image

  def callback(self,data):
    try:
      # camera turtlebot
      # np_arr = np.fromstring(data.data, np.uint8)
      # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      # cv_image = image_np
      # camera gazebo
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

      # lower mask (0-10) 
      lower_red = np.array([0,50,50])
      upper_red = np.array([10,255,255])
      mask0 = cv2.inRange(hsv, lower_red, upper_red)

      # upper mask (170-180)
      lower_red = np.array([170,50,50])
      upper_red = np.array([180,255,255])
      mask1 = cv2.inRange(hsv, lower_red, upper_red)
      
      mask = mask0+mask1

      output_hsv = hsv.copy()
      output_hsv[np.where(mask==0)] = 0

      cv2.imshow("Color Detected", np.hstack((cv_image,output_hsv)))

    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)