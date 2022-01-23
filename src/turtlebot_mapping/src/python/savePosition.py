#!/usr/bin/env python3
from socket import SO_ACCEPTCONN
from std_msgs.msg import String, Bool
import rospy
import numpy
from turtlebot3_msgs.msg import *
from actionlib_msgs.msg import *
import xml.etree.ElementTree as ET

number = 0
distance = 0.2
positionList = []

class SavePosition():

    def __init__(self):
        self.saved = False
        rospy.on_shutdown(self.shutdown)

            
    def save():
        message = rospy.wait_for_message("/tf",TFMessage)
        saved = SavePosition.saveToFile(message)
        return saved


    def saveToFile(message):
        global number        
        if SavePosition.checkMessage(message):
            f = open("position.txt","a")
            f.write(str(number) + "\n")
            f.write(str(message) + "\n\n")
            f.close()
            rospy.loginfo('position Saved as number ' + str(number))
            number = number + 1
            return True
        else: 
            rospy.loginfo('position already saved')         
            return False

    def shutdown(self):
        rospy.loginfo("SavePosition stop")

    def checkMessage(message):
        global positionList
        transformList = message.transforms
        for transformStamped in transformList:
            if transformStamped.child_frame_id == "base_footprint":
                position = transformStamped.transform.translation
                if not SavePosition.isInList(position):
                    positionList.append(position)
                    return True
                else:
                    return False

    def isInList(argposition):
        global positionList
        global distance
        for position in positionList:
            array1 = numpy.array((position.x,position.y,position.z))
            array2 = numpy.array((argposition.x,argposition.y,argposition.z))
            if(abs(numpy.linalg.norm(array1-array2)))<distance:
                return True

        return False    

def main():
    try:
        save = SavePosition()
        return save
        # SavePosition.save()
        # rospy.init_node('savePosition', anonymous=False)
    except rospy.ROSInterruptException:
        rospy.rospy.loginfo('Ctrl-C caught. Quitting')

if __name__ == '__main__':
    main()
