# Documentation of Autonomous Systems
## Setting up the Turtlebot
Firstly we updated the turtlebot to ros/noetic
https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup

Change the keyboard layout:
sudo dpkg-reconfigure keyboard-configuration

Setting up the network:
After connecting the raspberry pi to a screen, open 
$ cd /media/\$USER/writable/etc/netplan
$ sudo nano 50-cloud-init.yaml

change the WIFI-SSID to "labs@fhv.at"
and add the password after the colon vZDjRViutq9lSJ

afterwards check wifi connection by running: **ifconfig**

## Connection between host and raspberry
try to ping the host from the raspberry and vis versa
afterwards try to ssh from each other
export ROS_IP=ipofHost
export ROS_HOSTNAME=ipofRaspberry
export ROS_MASTER_URI=http://ipofHost:11311

try to start a roscore on the host and check if the raspberry can access the topic list
http://wiki.ros.org/ROS/NetworkSetup

If the topics are listed - run rostopic echo /topic
If the data is arraving on the host name run the pyhton script to firstly show the picture from the camera

Changed to WSL1 - working connection from the rosmaster on laptop to the rosnode on the robot

Running first version:
- connect via ssh to the turtlebot
- change the ~/.bashrc file so the ip address is correct (ROS_MASTER_URI=http://172.22.72.11 (laptop) ROS_HOSTNAME=172.22.73.10 (robot))
- run roslaunch turtlebot3_bringup turtlebot3_remote.launch on the laptop
- check connection to the master by running rostopic list
- run roslaunch turtlebot3_bringup turtlebot3_robot.launch on the robot
- run roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch on the laptop to steer the robot

## Camera
Adding a camera to the turtlebot burger:
Edit the file turtlebot3_burger.urdf.xacro file as well as the turtlebot3_burger.gazebo.xacro files
Add the camera from the waffle model to the burger model
Start up the OpenCV example to view the picture


## installing raspi-config
https://chuckmails.medium.com/enable-pi-camera-with-raspberry-pi4-ubuntu-20-10-327208312f6e



Questions:
1. ROS Core does start on the Raspberry but there are some packages missing???
2. can the core be run on the host? -- yes with wsl1 the connection is getting there
3. if running in WSL, how to access it from Raspberry - WSL1 is the solution

Next tasks:
1. try to update all packages on the Raspberry
2. don't forget catkin_make 
3. try to run core on host and connect Raspberry to it - done

Simulation tasks:
1. running core and gazebo simulation
2. running SLAM autonomously
3. save map
4. running a working camera and get the picture via opencv 


## Setup.bash file
export TURTLEBOT3_MODEL=burger
export GAZEBO_IP=127.0.0.1
export DISPLAY='OU-FF-IMZ-NB:0.0'
export LIBGL_ALWAYS_INDIRECT=0
export ROS_MASTER_URI=http://172.22.72.11:11311
