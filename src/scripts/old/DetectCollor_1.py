#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math



class image_converter:

  def __init__(self):

    self.image_pub = rospy.Publisher("image_topic_2",Image)
    # img = cv2.imread(self.image_pub)
    # k = 5
    # width = int((img.shape[1])/k)
    # height = int((img.shape[0])/k)
    # scaled = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    # Image = scaled

    # cv2.imwrite("/camera/rgb/image_raw", image, [int(cv2.IMWRITE_PNG_COMPRESSION), 4])
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback) #for Gazebo tourtlebot
    # self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",Image,self.callback) #for real robot image

    
  def get_scan(self):
    global SAMPLES
    global SAMPLES_VIEW
    scan = rospy.wait_for_message('scan', LaserScan)
    scan_filter = []
      
    # rospy.loginfo(scan)
    SAMPLES = len(scan.ranges)
    SAMPLES_VIEW = 360
    if SAMPLES_VIEW > SAMPLES:
        SAMPLES_VIEW = SAMPLES
    if SAMPLES_VIEW == 1 :
        scan_filter.append(scan.ranges[0])
    
    else: 
        scan_filter.extend(scan.ranges)
    for i in range(SAMPLES_VIEW):
        if scan_filter[i] == float('Inf'):
            scan_filter[i] = 3.5
        elif math.isnan(scan_filter[i]):
            scan_filter[i] = 0
    
    return scan_filter

  def callback(self,data):
    global tokenFound
    tokenFound = False
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
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
      data = np.asarray(output_hsv) # Data shape(1080, 1920, 3)

      # Algorithm center of the blob detection

      # Read image
      img = output_hsv

      # convert image to grayscale image
      gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

      # convert the grayscale image to binary image
      ret,thresh = cv2.threshold(gray_image,127,255,0)

      # calculate moments of binary image
      M = cv2.moments(thresh)

      # calculate x,y coordinate of center
      if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
      else:
        cX, cY = 0, 0

      # put text and highlight the center
      cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
      cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

      # display the image
      cv2.imshow("Image", img)
      

      #/ Algorithm: center of the blob detection
      
      # cv2.imshow("Color Detected", output_hsv)

      # Robot moving algorithm

      self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
      twist = Twist()
      
      lidar_distances = self.get_scan()
      min_distance = min(lidar_distances)
      N1_direction = min(lidar_distances[0:30])
      N2_direction = min(lidar_distances[330:360])
      N_direction = min(N1_direction, N2_direction)
      E_up_direction = min(lidar_distances[290:329])
      W_up_direction = min(lidar_distances[31:70])

      # Token finding
      def goToToken():
        tokenCount = 0
        if 0 < cY < 1000:
          print(f"Distance to center: {cX=} , {cY=}")
          # Avoidance of possible obstacles
          if N_direction < 0.1:
            if N1_direction-N2_direction>0.1:
              print("Avoid collision. Turn right")
              twist.linear.x = 0.0
              twist.angular.z = -0.1
              self._cmd_pub.publish(twist)
            elif N2_direction-N1_direction>0.1:
              print("Avoid collision. Turn left")
              twist.linear.x = 0.0
              twist.angular.z = 0.1
              self._cmd_pub.publish(twist)  
          # Getting to the center of token
          if cX <= 940:
            twist.linear.x = 0.07
            twist.angular.z = 0.05
            self._cmd_pub.publish(twist)
          elif cX > 980:
            twist.linear.x = 0.07
            twist.angular.z = -0.05
            self._cmd_pub.publish(twist)
          else:
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
        else:
          twist.linear.x = 0
          twist.angular.z = 0
          self._cmd_pub.publish(twist)
          tokenCount += 1
          print(tokenCount)
          if tokenCount > 20:
            tokenFound = False

      # /Token finding

      # Wallfollower
      def followWall():
        distance = 0.3
        if N_direction > distance and W_up_direction > distance and E_up_direction > distance:
            print("Find wall")
            twist.linear.x = 0.06
            twist.angular.z = -0.3
            self._cmd_pub.publish(twist)
        # Turning left if facing obstacle only at N sensor
        elif N_direction < distance and W_up_direction > distance and E_up_direction > distance:
            print("Turn left")
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            self._cmd_pub.publish(twist)
        
        elif N_direction < distance and W_up_direction < distance and E_up_direction < distance:
            print("Turn left")
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            self._cmd_pub.publish(twist)                
        elif N_direction > distance and W_up_direction > distance and E_up_direction < distance:
            print("Follow wall")
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)
        elif N_direction > distance and W_up_direction < distance and E_up_direction > distance:
            print("Find wall")
            twist.linear.x = 0.2
            twist.angular.z = -0.2
            self._cmd_pub.publish(twist)
        elif N_direction < distance and W_up_direction > distance and E_up_direction < distance:
            print("Turn left")
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            self._cmd_pub.publish(twist)
        elif N_direction < distance and W_up_direction < distance and E_up_direction > distance:
            print("Turn left")
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            self._cmd_pub.publish(twist)
        elif N_direction > distance and W_up_direction < distance and E_up_direction < distance:
            print("Find wall")
            twist.linear.x = 0.02
            twist.angular.z = -0.05
            self._cmd_pub.publish(twist)
        else:
            print("Spin")
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self._cmd_pub.publish(twist)
      # /Wallfollower

      if cX != 0 and cY != 0:
        tokenFound = True

      if tokenFound:
        goToToken()
      else:
        followWall()       


      # /Robot moving algorithm
      

    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


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