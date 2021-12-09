import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

LINEAR_VEL = 0.2
ANGULAR_VEL = 0.5
STOP_DISTANCE = 0.5
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
SAFE_DISTANCE = 0.2
WALLFOUND = True
SAMPLES = 0
SAMPLES_VIEW = 0

class FollowWall():

    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.follow()

    def get_scan(self):
        global SAMPLES
        global SAMPLES_VIEW
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []

        SAMPLES = len(scan.ranges)
        SAMPLES_VIEW = 360

        if SAMPLES_VIEW > SAMPLES:
            SAMPLES_VIEW = SAMPLES

        if SAMPLES_VIEW == 1 :
            scan_filter.append(scan.ranges[0])
        
        else: 
            scan_filter.extend(scan.ranges)


        for i in range(SAMPLES_VIEW):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def follow(self):
        global WALLFOUND
        global SAMPLES
        global SAMPLES_VIEW
        twist = Twist()
        turtlebot_moving = False

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            N1_direction = min(lidar_distances[0:30])
            N2_direction = min(lidar_distances[330:360])
            N_direction = min(N1_direction, N2_direction)
            W_direction = min(lidar_distances[60:120])
            S_direction = min(lidar_distances[150:210])
            E_up_direction = min(lidar_distances[290:320])
            E_mid_direction = min(lidar_distances[260:280])
            E_down_direction = min(lidar_distances[220:250])
            E_region = [E_up_direction, E_mid_direction, E_down_direction]
            E_direction = min(E_region)

            if not WALLFOUND : 
                #no wall found so continue straight
                print("WallNotFound")
                if N_direction > SAFE_DISTANCE:
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = True
                    rospy.loginfo('Distance of the obstacle : %f', min_distance)
                    WALLFOUND = False

                elif turtlebot_moving:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self._cmd_pub.publish(twist)
                        turtlebot_moving = False
                        rospy.loginfo('Stop!')
                        WALLFOUND = True       
                        print("I found a wall and will follow it")


            else : #found a wall and follow it
                # WALLFOUND = True
                safedistance = 0.2
                print("WALLFOUND")
                print(f"{N_direction=}, {E_direction=}")

                if N_direction < E_direction:
                    print("Obstacle upfront. Avoid collision")
                    twist.linear.x = 0.05
                    twist.angular.z = 0.5
                    self._cmd_pub.publish(twist)
                elif 0.25 > E_direction > 0.15 and 0.15 < N_direction < 0.5:
                    print("Avoid collision")
                    twist.linear.x = 0.05
                    twist.angular.z = 0.5
                    self._cmd_pub.publish(twist)
                elif 0.15 > E_direction and 0.15 > N_direction:
                    print("Go back")
                    twist.linear.x = -0.05
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                elif E_mid_direction < E_up_direction and E_mid_direction < E_down_direction:
                    print("Follow wall")
                    twist.linear.x = 0.4
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                elif E_up_direction-E_down_direction > 0.2:
                    print(E_up_direction-E_down_direction)
                    print("Turn right")
                    twist.linear.x = 0.1
                    twist.angular.z = -0.4     
                    self._cmd_pub.publish(twist)  
                elif E_down_direction-E_up_direction>0.2:
                    print(E_down_direction-E_up_direction)
                    print("Turn left")
                    twist.linear.x = 0.1
                    twist.angular.z = 0.4    
                    self._cmd_pub.publish(twist) 

def main():
    rospy.init_node('turlebot3_wallfollower')
    try:
        FollowWall()
    except rospy.ROSInterruptException:
        pass

if __name__ ==  '__main__':
    main()