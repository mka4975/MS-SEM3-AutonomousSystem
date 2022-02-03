import roslaunch
import rospy
import os

rospy.init_node('navigation', anonymous=False)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/michael/turtlebot_ws/src/turtlebot_mapping/launch/navigation.launch"])
launch.start()
rospy.loginfo("started")

os.system("rosservice call /global_localization")

rospy.spin()