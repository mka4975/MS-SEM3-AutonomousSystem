import roslaunch
import rospy

rospy.init_node('simulation', anonymous=False)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/michael/turtlebot_ws/src/turtlebot_mapping/launch/simulation.launch"])
launch.start()
rospy.loginfo("started")

rospy.spin()