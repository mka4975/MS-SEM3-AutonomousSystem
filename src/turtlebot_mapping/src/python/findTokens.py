#!/usr/bin/env python
from __future__ import print_function
from atexit import unregister

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import String, Bool

from turtlebot3_msgs.msg import Sound

import inputNumberOfTokens
from savePosition2 import SavePosition
from getScan import get_scan
from tokenOrder import tokenOrder

# global variables 
numberOfTokens = 0
numberOfFoundTokens = 0
allTokensFound = False
cX = 0
cY = 0
tokenFound=False

mapCompleted = False


class findTags:
  global numberOfTokens
  global numberOfFoundTokens
  global allTokensFound

  def start(self, argnumberOfTokens):
    global numberOfTokens 
    numberOfTokens = argnumberOfTokens
    self.bridge = CvBridge()
    self.map_sub = rospy.Subscriber("mapCompleted", Bool, self.mapCompletedCallback)
    #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.imageCallback) #for Gazebo turtlebot
    self.cmd_sub = rospy.Subscriber('scan', LaserScan, self.scanCallback)
    self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.imageCallback) #for real robot image


  def imageCallback(self,data):
    global cX
    global cY
    global tokenFound
    tokenFound = False
    
    try:
      #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") #for the simulation
      np_arr = np.fromstring(data.data, np.uint8) # for the turtlebot
      cv_image = self.bridge.compressed_imgmsg_to_cv2(np_arr, "bgr8") #for the turtlebot
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
      # cv2.imshow("Image", img)
      

      #/ Algorithm: center of the blob detection
      
      cv2.imshow("Color Detected", output_hsv)
    except CvBridgeError as e:
      print(e)

    
    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    

  def scanCallback(self,data):
    global allTokensFound
    global tokenFound
    # Robot moving algorithm
    self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self._sound_pub = rospy.Publisher('sound', Sound, queue_size=10)
    
    
    lidar_distances = get_scan(data)
    N1_range = lidar_distances[5:30]
    N2_range = lidar_distances[330:355]
    N_range = N1_range + N2_range
    E_up_range = lidar_distances[290:329]
    E_mid_range = lidar_distances[270:280]
    W_up_range = lidar_distances[31:70]

    N1_direction = min(i for i in N1_range if i > 0)
    N2_direction = min(i for i in N2_range if i > 0)
    N_direction = min(i for i in N_range if i > 0)
    E_up_direction = min(i for i in E_up_range if i > 0)
    E_mid_direction = min(i for i in E_mid_range if i > 0)
    W_up_direction = min(i for i in W_up_range if i > 0)

    if allTokensFound: 
        stopRobot()
        self.map_sub.unregister()
        self.image_sub.unregister()
        self.cmd_sub.unregister()
        tokenOrder.main(numberOfTokens)
        print("All done")
        rospy.signal_shutdown("alldone")
    else :
      if cX != 0 and cY >= 300:
        tokenFound = True

      if tokenFound:
        self.goToToken(N1_direction,N2_direction,N_direction,E_up_direction,W_up_direction)
      else:
        self.followWall(N_direction,E_up_direction,E_mid_direction,W_up_direction)   

      # Token finding
  def goToToken(self,N1_direction,N2_direction,N_direction,E_up_direction,W_up_direction):
    twist = Twist()
    distance = 0.3
    if cY < 680:
      print(f"Distance to center: {cX=} , {cY=}")

      # Avoidance of possible obstacles
      print(f"{N_direction=}, {N1_direction=}, {N2_direction=}")
      if N_direction > 0 and N_direction < distance and E_up_direction > W_up_direction:
          print("Avoid collision. Turn right")
          twist.linear.x = 0.1
          twist.angular.z = -0.3
          self._cmd_pub.publish(twist)
      elif N_direction > 0 and N_direction < distance and E_up_direction < W_up_direction:
        print("Avoid collision. Turn left")
        twist.linear.x = 0.1
        twist.angular.z = 0.3
        self._cmd_pub.publish(twist)  
      else:
        # Getting to the center of token
        if cX <= 700:
          print("twistLeft")
          twist.linear.x = 0.05
          twist.angular.z = 0.1
          self._cmd_pub.publish(twist)
        elif cX > 750:
          print("twistRight")
          twist.linear.x = 0.05
          twist.angular.z = -0.1
          self._cmd_pub.publish(twist)            
        else:
          print("Forward")
          twist.linear.x = 0.15
          twist.angular.z = 0.0
          self._cmd_pub.publish(twist)
    else:
      print("Stop")
      twist.linear.x = 0.0
      twist.angular.z = 0.0
      self._cmd_pub.publish(twist)
      foundToken(self)

      # Wallfollower
  def followWall(self,N_direction,E_up_direction,E_mid_direction,W_up_direction):
    distance = 0.3
    safeDistance = 0.2
    twist = Twist()
    print(f"{N_direction=}, {W_up_direction=}, {E_up_direction=}, {E_mid_direction=}")
    if N_direction > distance and W_up_direction > distance and E_up_direction > distance:
        print("Turn right 1")
        twist.linear.x = 0.05
        twist.angular.z = -0.4
        self._cmd_pub.publish(twist)
    # Turning left if facing obstacle only at N sensor
    elif N_direction < distance and W_up_direction > distance and E_up_direction > distance:
        print("Turn left 1")
        twist.linear.x = 0.0
        twist.angular.z = 0.3
        self._cmd_pub.publish(twist)
    
    elif N_direction < distance and W_up_direction < distance and E_up_direction < distance:
        print("Turn left 2")
        twist.linear.x = 0.0
        twist.angular.z = 0.3
        self._cmd_pub.publish(twist)                
    elif N_direction > distance and W_up_direction > distance and E_up_direction < distance and E_mid_direction > distance:
        print("Follow wall 1a")
        twist.linear.x = 0.05
        twist.angular.z = -0.1
        self._cmd_pub.publish(twist)
    elif N_direction > distance and W_up_direction > distance and E_up_direction < distance and E_mid_direction > safeDistance:
        print("Follow wall 1b")
        twist.linear.x = 0.1
        twist.angular.z = -0.0
        self._cmd_pub.publish(twist)
    elif N_direction > distance and W_up_direction > distance and E_up_direction < distance and E_mid_direction < safeDistance:
        print("Follow wall 1c")
        twist.linear.x = 0.05
        twist.angular.z = 0.02
        self._cmd_pub.publish(twist)
    elif N_direction > distance and W_up_direction < distance and E_up_direction > distance and E_mid_direction > distance:
        print("Turn right 2")
        twist.linear.x = 0.1
        twist.angular.z = -0.4
        self._cmd_pub.publish(twist)
    elif N_direction > distance and W_up_direction < distance and E_up_direction > distance:
        print("Find wall")
        twist.linear.x = 0.015
        twist.angular.z = -0.4
        self._cmd_pub.publish(twist)
    elif N_direction < distance and W_up_direction > distance and E_up_direction < distance:
        print("Turn left 3")
        twist.linear.x = 0.0
        twist.angular.z = 0.3
        self._cmd_pub.publish(twist)
    elif N_direction < distance and W_up_direction < distance and E_up_direction > distance:
        print("Turn left 4")
        twist.linear.x = 0.0
        twist.angular.z = 0.3
        self._cmd_pub.publish(twist)
    elif N_direction > distance and W_up_direction < distance and E_up_direction < distance and E_mid_direction > safeDistance:
        print("Follow wall 2a")
        twist.linear.x = 0.1
        twist.angular.z = -0.05
        self._cmd_pub.publish(twist)
    elif N_direction > distance and W_up_direction < distance and E_up_direction < distance and E_mid_direction < safeDistance:
        print("Follow wall 2b")
        twist.linear.x = 0.1
        twist.angular.z = 0.01
        self._cmd_pub.publish(twist)
    else:
        print("Spin")
        twist.linear.x = 0.0
        twist.angular.z = -0.5
        self._cmd_pub.publish(twist)
      # /Wallfollower
      # /Robot moving algorithm
    
  # fallback if the map completion doesn't work
  def mapCompletedCallback(self,data):
    global mapCompleted
    if(data.data == True):
      print("Map completed please save it!")
      mapCompleted = True


def foundToken(self):
  global numberOfTokens
  global numberOfFoundTokens
  global tokenFound
  global allTokensFound
  if numberOfFoundTokens < int(numberOfTokens) :
    saved = SavePosition.save(self)
    #saved = True - new Token - saved to file
    #saved = False - Token already in file
    
    
    if saved:
      tokenFound = False
      numberOfFoundTokens+=1
      print("token Found")
      print("numberOfFoundTokens: " + str(numberOfFoundTokens))
      print("numberOfTokens: " + str(numberOfTokens))
      print("TokenSaved: " + str(saved))
    else:
      tokenFound = False

  else:
    finish_pub = rospy.Publisher("globalState", String, queue_size=10)
    msg = String("All Tokens Found")
    finish_pub.publish(msg)
    allTokensFound = True

def main():
  global numberOfTokens
  numberOfTokens = inputNumberOfTokens.main()
  ic = findTags()
  rospy.init_node('findTokens', anonymous=False,disable_signals=True)
  
  findTags.start(ic,numberOfTokens)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    stopRobot()
    print("Shutting down")
  cv2.destroyAllWindows()

def stopRobot():
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    publisher = rospy.Publisher('twist', Twist, queue_size=10) 
    publisher.publish(twist)


if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt:
    stopRobot()
    print("Shutting down")
  cv2.destroyAllWindows()