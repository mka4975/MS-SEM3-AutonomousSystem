import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch_files = ["/home/michael/turtlebot_ws/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_slam.launch"]

launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

launch.start()

#TODO: run pyhton file to follow Wall and find tags

rospy.loginfo("started")

rospy.spin()