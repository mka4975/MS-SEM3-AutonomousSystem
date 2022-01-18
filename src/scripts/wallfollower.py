import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

LINEAR_VEL = 0.2
ANGULAR_VEL = 0.5
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
SAFE_DISTANCE = 0.3
WALLFOUND = False
SAMPLES = 0
SAMPLES_VIEW = 0

stop = False


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
        
        # rospy.loginfo(scan)

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
            S_direction = min(lidar_distances[150:210])
            E_up_direction = min(lidar_distances[290:320])
            E_mid_direction = min(lidar_distances[260:280])
            E_down_direction = min(lidar_distances[220:250])
            E_region = [E_up_direction, E_mid_direction, E_down_direction]
            E_direction = min(E_region)
            W_up_direction = min(lidar_distances[40:70])
            W_mid_direction = min(lidar_distances[80:110])
            W_down_direction = min(lidar_distances[120:150])
            W_region = [W_up_direction, W_mid_direction, W_down_direction]
            W_direction = min(W_up_direction, W_mid_direction, W_down_direction)



            # if N_direction < distance:
            #     twist.angular.z=0.5
            #     self._cmd_pub.publish(twist)
            #     turtlebot_moving = True



            if not WALLFOUND : 
                #no wall found so continue straight
                if N_direction > SAFE_DISTANCE:
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = -0.01
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = True
                    rospy.loginfo('Distance in the N direction: %f', N_direction)
                    WALLFOUND = False

                elif turtlebot_moving:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self._cmd_pub.publish(twist)
                        turtlebot_moving = False
                        rospy.loginfo('Stop!')
                        WALLFOUND = True       
                        print("I found a wall and will follow it")
                        turn_delay = False


            else : #found a wall and follow it
                WALLFOUND = True


                if SAFE_DISTANCE > N_direction:
                    if E_up_direction-W_up_direction>0.1 and not turn_delay:
                        print("Avoid collision. Turn right")
                        twist.linear.x = 0.0
                        twist.angular.z = -0.2
                        self._cmd_pub.publish(twist)
                    elif W_up_direction-E_up_direction>0.1 and not turn_delay:
                        print("Avoid collision. Turn left")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.2
                        self._cmd_pub.publish(twist)  
                    else:
                        print("Go out of the corner")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.4
                        self._cmd_pub.publish(twist)
                        turn_delay = True

                elif 0.2 > (E_direction-W_direction) > -0.2 and N_direction >= SAFE_DISTANCE:
                    turn_delay = False
                    if E_mid_direction < E_up_direction and E_mid_direction < E_down_direction:
                        print("Follow wall")
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0
                        self._cmd_pub.publish(twist)
                    elif E_up_direction-E_down_direction > 0.1 and N_direction >= 0.2:
                        print("Turn right")
                        twist.linear.x = 0.1
                        twist.angular.z = -0.4     
                        self._cmd_pub.publish(twist)  
                    elif E_down_direction-E_up_direction>0.1 and N_direction >= 0.2:
                        print("Turn left")
                        twist.linear.x = 0.1
                        twist.angular.z = 0.4  
                        self._cmd_pub.publish(twist) 
                    
                # if   N_direction < safedistance and W_up_direction < safedistance and E_up_direction < safedistance:
                #     twist.linear.x = 0.0
                #     twist.angular.z = 0.3
                #     self._cmd_pub.publish(twist)
                #     print("Turn left")
                # elif N_direction< safedistance and W_up_direction  < safedistance and E_up_direction > safedistance:
                #     twist.linear.x = 0.1
                #     twist.angular.z = 0.01
                #     self._cmd_pub.publish(twist)
                #     print("b")
                # elif N_direction< safedistance and W_up_direction  > safedistance and E_up_direction < safedistance:
                #     twist.linear.x = 0.0
                #     twist.angular.z = 0.3
                #     self._cmd_pub.publish(twist)
                #     print("c")
                # elif N_direction< safedistance and W_up_direction  > safedistance and E_up_direction > safedistance:
                #     twist.linear.x = 0.0
                #     twist.angular.z = 0.3
                #     self._cmd_pub.publish(twist)
                #     print("d")
                # elif N_direction> safedistance and W_up_direction  < safedistance and E_up_direction < safedistance:
                #     twist.linear.x = 0.1
                #     twist.angular.z = 0.01
                #     self._cmd_pub.publish(twist)
                #     print("e")
                # elif N_direction> safedistance and W_up_direction  < safedistance and E_up_direction > safedistance:
                #     twist.linear.x = 0.1
                #     twist.angular.z = 0.01
                #     self._cmd_pub.publish(twist)
                #     print("f")
                # elif N_direction> safedistance and W_up_direction  > safedistance and E_up_direction < safedistance:
                #     twist.linear.x = 0.1
                #     twist.angular.z = 0.0
                #     self._cmd_pub.publish(twist)
                #     print("g")
                # elif N_direction> safedistance and W_up_direction  > safedistance and E_up_direction > safedistance:
                #     twist.linear.x = 0.1
                #     twist.angular.z = 0.01
                #     self._cmd_pub.publish(twist)
                #     print("h")
 

            

                # left_lidar_samples_ranges = -(SAMPLES_VIEW//2 + SAMPLES_VIEW % 2)
                # right_lidar_samples_ranges = SAMPLES_VIEW//2
                
                # left_lidar_samples = lidar_distances[left_lidar_samples_ranges:]
                # right_lidar_samples = lidar_distances[:right_lidar_samples_ranges]
            
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
            while not stop : 
                follow()
        except rospy.ROSInterruptException:
            pass


    def stop():
        global stop
        stop = True

    if __name__ ==  '__main__':
        main()