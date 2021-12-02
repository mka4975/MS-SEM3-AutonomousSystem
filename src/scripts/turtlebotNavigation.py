import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch_files = ["/home/michael/turtlebot_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/Labyrinth_easy.launch",
                "/home/michael/turtlebot_ws/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_gmapping.launch",
                "/home/michael/turtlebot_ws/src/turtlebot3/turtlebot3_navigation/launch/turtlebot3_navigation.launch",
                "/home/michael/turtlebot_ws/src/turtlebot3/turtlebot3_bringup"]

launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

launch.start()

rospy.loginfo("started")

rospy.spin()