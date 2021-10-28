#!/usr/bin/env python
from math import ldexp
from sys import modules
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

SAMPLES = 0
SAMPLES_VIEW = 0
   
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('cmd_vel', anonymous=True)
move_cmd = Twist()
move_cmd.linear.x = -1.22

now = rospy.Time.now()
rate = rospy.Rate(1000)

while not rospy.is_shutdown():
   scan = rospy.wait_for_message('scan', LaserScan)
   scan_filter = []
   SAMPLES = len(scan.ranges)
   SAMPLES_VIEW = 360

   if SAMPLES_VIEW > SAMPLES:
      SAMPLES_VIEW = SAMPLES

   if SAMPLES_VIEW == 1 :
      scan_filter.append(scan.ranges[0])
        
   else: 
      scan_filter.extend(scan.ranges)
   lidar_distances = scan_filter
   rospy.loginfo(lidar_distances)
   min_distance = min(lidar_distances)
   rospy.loginfo(move_cmd)
   pub.publish(move_cmd)
   rate.sleep() 

