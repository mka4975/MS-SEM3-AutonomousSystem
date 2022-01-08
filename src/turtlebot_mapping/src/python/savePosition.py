#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf
import geometry_msgs.msg
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class SavePosition():

    def __init__(self):
        self.saved = False
        rospy.on_shutdown(self.shutdown)

    def save(self):
        self.saved = True

        listener = tf.TransformListener()

        (trans,rot)=listener.lookupTransform("tf_echo","/map","/base_link",rospy.Time(0))

        if trans != None :
            rospy.loginfo(trans)
            rospy.loginfo(rot)
            return True
        else:
            return False

    def shutdown(self):
        rospy.loginfo("Stop")
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        saver = SavePosition()
        rospy.init_node('savePosition', anonymous=False)
        success = saver.save()

        if success:
            rospy.rospy.loginfo('Position Saved')
        else:
            rospy.loginfo('Position could not be saved')
        
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.rospy.loginfo('Ctrl-C caught. Quitting')
