#!/usr/bin/env python3
from socket import SO_ACCEPTCONN
from std_msgs.msg import String, Bool
import rospy
from turtlebot3_msgs.msg import *
from actionlib_msgs.msg import *
import xml.etree.ElementTree as ET

number = 0

class SavePosition():

    def __init__(self):
        self.saved = False
        rospy.on_shutdown(self.shutdown)

    def saveToFile (self, message):
        global number        
        if self.checkMessage(message):
            f = open("position.txt","a")
            f.write(str(number) + "\n")
            f.write(str(message) + "\n\n")
            f.close()
            rospy.loginfo('position Saved as number ' + str(number))
        else: 
            f = open("position.txt","a")
            f.write(str(number) + "\n")
            f.write("not successfull" + "\n\n")
            f.close()
        number = number + 1
        return True           


    def nodeCallback(self, message):
        global number
        success = False

        if message.data == True:
            success = rospy.Subscriber("tf", TFMessage, self.tfCallback)
        rospy.loginfo('heard on tokenFound' + str(message))
        SavePosition.saveToFile(message)
        return success

        
    def save(self):
        self.saved = True
        rospy.Subscriber("tokenFound", Bool, self.nodeCallback)
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stop")

    def checkMessage(self,message):
        x = ""
        y = ""
        z = ""
        x = (str(message).split("x: "))[1].split("\n"[0])
        y = (str(message).split("y: "))[1].split("\n"[0])
        z = (str(message).split("z: "))[1].split("\n"[0])
        
        rospy.loginfo('values' + str(x) + str(y) + str(z))


def main():
    try:
        saver = SavePosition()
        rospy.init_node('savePosition', anonymous=False)
        saver.save()

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.rospy.loginfo('Ctrl-C caught. Quitting')

if __name__ == '__main__':
    main()
