#!/usr/bin/env python
from __future__ import print_function
from atexit import unregister
import glob
import re
from shutil import ExecError
from tkinter import E

from matplotlib.pyplot import xlim

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
import findTokens
import goToPosition

STATE_OFF             = 0
STATE_GLOBAL_LOCALIZE = False
STATE_MOW             = 2

tokensToFind = []
tokenAproaching = 0
gotPositions = False
xList = []
yList = []

class navigation:
  global STATE_OFF             
  global STATE_GLOBAL_LOCALIZE 
  global STATE_MOW             
  global tokensToFind
  global tokenAproaching


  def main():
    rospy.init_node('navigationMove', anonymous=False)
    particle_sub = rospy.Subscriber("particlecloud", PoseArray, navigation.particle_handler) 
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    
    while not rospy.is_shutdown():
      if not STATE_GLOBAL_LOCALIZE:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 1
        cmd_vel.publish(twist)
      else:
        navigation.getPositionsFromFile()
        navigation.goToPostitions()
        

    rospy.spin()

  def particle_handler(msg):
    global STATE_GLOBAL_LOCALIZE
    if STATE_GLOBAL_LOCALIZE == False:
      xmin = 99999
      xmax = 0
      ymin = 99999
      ymax = 0
      for pose in msg.poses:
        xmin = min(xmin, pose.position.x)
        xmax = max(xmax, pose.position.x)
        ymin = min(ymin, pose.position.y)
        ymax = max(ymax, pose.position.y)
      xr = abs(xmax-xmin)
      yr = abs(ymax-ymin)
      print(xr, yr) 
      if float(yr) < 1: 
        if float(xr) < 1:
          STATE_GLOBAL_LOCALIZE = True
    else:
      return

  def goToPostitions():
    global xList
    global yList
    new_lst = [list(x) for x in zip(xList, yList)]
    goToPosition.main(new_lst)

  def getPositionsFromFile():
    global gotPositions
    global xList
    global yList
    if not gotPositions:
      translation = "translation"
      try:
        f = open("position.txt","r")
        lines = f.readlines()
        
        for i in range(0,len(lines)):
          line = lines[i]
          if translation in line:
            x = lines[i+1]
            y = lines[i+2]
            x = x.split("x: ",1)[1]
            x = x.strip()
            xList.append(float(x))
            y = y.split("y: ",1)[1]
            y = y.strip()
            yList.append(float(y))

        f.close
        print(xList)
        print(yList)
        gotPositions = True
      except Exception:
        return

if __name__ == '__main__':
  try:
    navigation.main()
  except KeyboardInterrupt:
    print("Shutting down")