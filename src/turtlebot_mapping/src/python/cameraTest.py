#!/usr/bin/env python
from __future__ import print_function
import glob
from logging import getLoggerClass

import sys
import rospy
import cv2
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

numberOfFoundTokens = 0
numberOfTokens = 0
class findTags:
  global numberOfFoundTokens
  global numberOfTokens

  def main(self, argnumberOfTokens):
    global numberOfTokens 
    numberOfTokens = argnumberOfTokens
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
    # img = cv2.imread(self.image_pub)
    # k = 5
    # width = int((img.shape[1])/k)
    # height = int((img.shape[0])/k)
    # scaled = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    # Image = scaled

    # cv2.imwrite("/camera/rgb/image_raw", image, [int(cv2.IMWRITE_PNG_COMPRESSION), 4])
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback) #for Gazebo tourtlebot
    self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback) #for real robot image

    
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
    global numberOfTokens
    tokenFound = False
    
    try:
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data,"bgr8")
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
        
    except CvBridgeError as e:
        print(e)

def main(numberOfTokens):
    ic = findTags()
    
    findTags.main(ic,numberOfTokens)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

        # /Token finding
    def tokenFound(tokenCount):
        msg = Bool(True)
        self.tokenFound.publish(msg)
        rospy.sleep(3)
        # publish position of robot to topic tokenFound
        # if tokenCount >= numberOfTokens
        # check if map is complete
        # if map complete
            # save map and stop robot
        # if map not complete
            # go to open space
        # else:
        # continue finding tokens


if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    main(sys.argv)