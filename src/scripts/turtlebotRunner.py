#!/usr/bin/python
# from numpy import conj
# import roslaunch
import rospy
from wallfollower import FollowWall
from wallfollower2 import WallFollowerTwo
from turtlebotMaphandler import TurtlebotMapHandler
import time
import threading as t


mapFinished = False
class TurtlebotRunner :
    
    def main():
        rospy.init_node('TurtlebotRunner')
        global mapFinished
        runner = TurtlebotRunner()
        threadCreated = False
        try:
            print("Creating Map")
            while not rospy.is_shutdown():
                if not mapFinished:
                    if not threadCreated : 
                        t.Thread(target=runner.createMap).start()
                        threadCreated = True
                # stop turtlebot
                # save map
                

        except rospy.ROSInterruptException:
            pass

    def createMap(arg):
        print("here")
        global mapFinished
        t.Thread(target = WallFollowerTwo.main()).start
        t.Thread(TurtlebotMapHandler.isMapFinished()).start
        

  

    def stop():
        WallFollowerTwo.stop()

    # def saveMap():
                
    #     launch_files = ["rosrun map_server map_saver -f mymap"]

    #     launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

    #     launch.start()


if __name__ == '__main__':
    TurtlebotRunner.main()
