#!/usr/bin/env python3
from socket import SO_ACCEPTCONN
from std_msgs.msg import String, Bool
import rospy
import numpy
from turtlebot3_msgs.msg import *
from actionlib_msgs.msg import *
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

number = 0
distance = 0.2
positionList = []

class SavePosition():

    def __init__(self):
        self.saved = False
        rospy.on_shutdown(self.shutdown)

            
    def save(self):
        message = rospy.wait_for_message("/odom",Odometry)
        saved = SavePosition.saveToFile(self,message)
        return saved


    def saveToFile(self,message):
        global number        
        if SavePosition.checkMessage(message):
            try:
                f = open("position.txt","a")
                f.write(str(number) + "\n")
                f.write(str(message.pose.pose) + "\n\n")
                f.close()
            except Exception:
                print(Exception)
            rospy.loginfo('position Saved as number ' + str(number))
            number = number + 1
            self._sound_pub.publish(1)
            return True
        else: 
            rospy.loginfo('position already saved')         
            return False

    def shutdown(self):
        rospy.loginfo("SavePosition stop")

    def checkMessage(message):
        global positionList
        position = message.pose.pose
        # if len(positionList)==0:
        #     positionList.append(position)
        #     return True
        if not SavePosition.isInList(position):
            positionList.append(position)
            return True
        else:
            return False

    def isInList(argposition):
        global positionList
        global distance
        for position in positionList:
            array1 = numpy.array((position.position.x,position.position.y,position.position.z))
            array2 = numpy.array((argposition.position.x,argposition.position.y,argposition.position.z))
            realDistance = abs(numpy.linalg.norm(array1-array2))
            print(realDistance)
            if realDistance<distance:
                return True

        return False    

def main():
    try:
        save = SavePosition()
        return save
    except rospy.ROSInterruptException:
        rospy.rospy.loginfo('Ctrl-C caught. Quitting')
