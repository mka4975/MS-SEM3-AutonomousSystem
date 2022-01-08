import roslaunch
import rospy
import tagInput

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

# launch_files = ["rosrun map_server map_saver"]

# launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

# launch.start()
rospy.loginfo("started")

globalTagNumbers = tagInput.main()
rospy.loginfo("global Tag numbers saved")

rospy.spin()