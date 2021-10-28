import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

LINEAR_VEL = 0.22
ANGULAR_VEL = 0.5
STOP_DISTANCE = 0.15
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
WALLFOUND = False
SAMPLES = 0
SAMPLES_VIEW = 0

class FollowWall():

    def __init__(self):
        global WALLFOUND
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.follow()

    def get_scan(self):
        global SAMPLES
        global SAMPLES_VIEW
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        rospy.loginfo(scan)

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

            if not WALLFOUND : 
                #no wall found so continue straight
                if min_distance > SAFE_STOP_DISTANCE:
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

            else : #found a wall and follow it
                minleft = 0
                minright = 0
                minfront = 0

                left_lidar_samples_ranges = -(SAMPLES_VIEW//2 + SAMPLES_VIEW % 2)
                right_lidar_samples_ranges = SAMPLES_VIEW//2
                
                left_lidar_samples = lidar_distances[left_lidar_samples_ranges:]
                right_lidar_samples = lidar_distances[:right_lidar_samples_ranges]
            
                # print("I found a wall and will follow it")

                # while WALLFOUND:
                #     difference = sum(left_lidar_samples) - sum(right_lidar_samples)

                #     twist.linear.x = LINEAR_VEL*0.1
                #     wallInFront = True
                #     if lidar_distances[0] == 3.5 and lidar_distances[SAMPLES_VIEW] == 3.5 :
                #         wallInFront = False

                #     if wallInFront : 
                #         if difference < 0 :
                #             twist.angular.z = -ANGULAR_VEL
                #         else :
                #             twist.angular.z = ANGULAR_VEL
                #     else : 
                #         twist.angular.z = 0       
                #     self._cmd_pub.publish(twist)
                
        

def main():
    rospy.init_node('turlebot3_wallfollower')
    try:
        FollowWall()
    except rospy.ROSInterruptException:
        pass

if __name__ ==  '__main__':
    main()