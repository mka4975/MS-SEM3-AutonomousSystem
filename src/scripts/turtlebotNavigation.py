import roslaunch
import rospy
from turtlebotRunner import TurtlebotRunner
import time


rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

# cli_args1 =

launch_files = ["/home/michael/turtlebot_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/Labyrinth_easy.launch",
                "/home/michael/turtlebot_ws/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_slam.launch"]

#/home/michael/turtlebot_ws/src/turtlebot3/turtlebot3_bringup"

launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

launch.start()

#TurtlebotRunner.main()

rospy.loginfo("started")

rospy.spin()