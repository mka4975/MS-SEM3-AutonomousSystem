# Documentation of Autonomous Systems
## Setting up the Turtlebot
Firstly we updated the turtlebot to ros/noetic
https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup

Change the keyboard layout:
sudo dpkg-reconfigure keyboard-configuration

Setting up the network:
After connecting the raspberry pi to a screen, open 

\$ cd /media/\$USER/writable/etc/netplan

\$ sudo nano 50-cloud-init.yaml

change the WIFI-SSID to "labs@fhv.at" and add the password after the colon vZDjRViutq9lSJ

afterwards check wifi connection by running: **ifconfig**

<br>

## Setup.bash file for robot
export TURTLEBOT3_MODEL=burger
export ROS_HOSTNAME=10.0.0.7
export ROS_IP=10.0.0.14
export ROS_MASTER_URI=http://10.0.0.14:11311


## Setup.bash file for pc
export TURTLEBOT3_MODEL=burger
export GAZEBO_IP=127.0.0.1
export DISPLAY='OU-FF-IMZ-NB:0.0'
export LIBGL_ALWAYS_INDIRECT=0
export ROS_MASTER_URI=http://172.22.72.11:11311
 
<br>

## Connection between host and raspberry
try to ping the host from the raspberry and vis versa
afterwards try to ssh from each other
export ROS_IP=ipofRaspberry
export ROS_HOSTNAME=ipofHost
export ROS_MASTER_URI=http://ipofHost:11311

try to start a roscore on the host and check if the raspberry can access the topic list
http://wiki.ros.org/ROS/NetworkSetup

If the topics are listed - run rostopic echo /topic
If the data is arriving on the host name run the python script to firstly show the picture from the camera

Changed to WSL1 - working connection from the rosmaster on laptop to the rosnode on the robot

<br>

## Running first version:
- connect via ssh to the turtlebot
- change the ~/.bashrc file so the ip address is correct (ROS_MASTER_URI=http://172.22.72.11 (laptop) ROS_HOSTNAME=172.22.72.11(laptop))
- run roscore on the laptop
- check connection to the master by running rostopic list on thr robot
- run roslaunch turtlebot3_bringup turtlebot3_robot.launch on the robot
- run roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch on the laptop to steer the robot

## Camera
Adding a camera to the turtlebot burger in the simulation:
Edit the file turtlebot3_burger.urdf.xacro file as well as the turtlebot3_burger.gazebo.xacro files
Add the camera from the waffle model to the burger model
Start up the OpenCV example to view the picture

**Camera on robot**
To get the image from the camera on the robot there needs to be a node running on the robot to publish the image from the raspi camera:

run *roslaunch raspicam_node camerav1_1280x720.launch on robot*

To view the image on the host:

run *rqt_image_view*

<br>

## Navigation
### Creation of a map
To create a map of the surrounding of the robot run the following command on the host:

*roslaunch turtlebot3_slam turtlebot3_slam.launch*

Also run the wallfollower as well as the saving tag program

The robot then starts navigating trough the labyrinth and saves the tags position in an array 

After the map is finished run:
rosrun map_server map_saver

Then the map can be loaded and navigated trough with
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/michael/turtlebot_ws/map.yaml

## TODO
- wallfollower and tag finder
- save tag position
- check map completion
