#!/usr/bin/env python

from turtle import pos
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from turtlebot3_msgs.msg import Sound


class GoToPose():
    def __init__(self):

        self.goal_sent = False

        rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                    Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


def main(positionList,target):
    currentTarget = target
    try:
        positionToGo = positionList[currentTarget]
        navigator = GoToPose()

        position = {'x': positionToGo[0], 'y': positionToGo[1]}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
            if currentTarget < len(positionList):
                currentTarget += 1
            return True
        else:
            rospy.loginfo("The base failed to reach the desired pose")
            return False

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")


if __name__ == '__main__':
    rospy.init_node('goToPos', anonymous=False,disable_signals=True)
    position1 = [[-1,-1],[1,1]]
    main(position1)