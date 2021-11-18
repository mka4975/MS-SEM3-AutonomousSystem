import rospy
from package_turtlebot_ogu.msg import Lidar
from geometry_msgs.msg import Twist

class WFMachine():

    STATE = {'FIND_WALL': 0,
             'TURN_LEFT': 1,
             'FOLLOW_WALL' : 2}

    def __init__(self):
        self.__STATE = WFMachine.STATE['FIND_WALL']
        self.__twist = Twist()
        self.__lidar = Lidar()
        self.__teleop_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    @property
    def CURRENT_STATE(self):
        return self.__STATE
    
    @CURRENT_STATE.setter
    def CURRENT_STATE(self, state):
        self.__STATE = state

    @property
    def lidar_value_front(self):
        return self.__lidar.Front.Value
    
    @property
    def lidar_value_front_left(self):
        return self.__lidar.Front_Left.Value
    
    @property
    def lidar_value_left(self):
        return self.__lidar.Left.Value

    @property
    def lidar_value_front_right(self):
        return self.__lidar.Front_Right.Value

    @property
    def lidar_value_right(self):
        return self.__lidar.Right.Value

    @property 
    def lidar_values(self):
        return self.__lidar

    @lidar_values.setter
    def lidar_values(self, ld):
        self.__lidar = ld
    
    def __find_wall(self):
        self.__twist.linear.x = 0.1
        self.__twist.angular.z = 0.01
        self.publish(self.__twist)

    def __turn_left(self):
        self.__twist.angular.z = 0.3
        self.publish(self.__twist)

    def __follow_wall(self):
        self.__twist.linear.x = 0.1
        self.__twist.angular.z = 0.0
        self.publish(self.__twist)
    
    def publish(self, tw):
        self.__teleop_pub.publish(tw)

    def determine_state(self):
        ''' set value '''
        distance = 0.5

        ''' Got 3 Values which can be either be greater then the distance or smaller -> 2 possibilities and 3 values -> 8 conditions '''

        if   self.lidar_value_front < distance and self.lidar_value_front_left < distance and self.lidar_value_front_right < distance:
            self.CURRENT_STATE = WFMachine.STATE['TURN_LEFT']
        
        elif self.lidar_value_front < distance and self.lidar_value_front_left < distance and self.lidar_value_front_right > distance:
            self.CURRENT_STATE = WFMachine.STATE['FIND_WALL']
        
        elif self.lidar_value_front < distance and self.lidar_value_front_left > distance and self.lidar_value_front_right < distance:
            self.CURRENT_STATE = WFMachine.STATE['TURN_LEFT']

        elif self.lidar_value_front < distance and self.lidar_value_front_left > distance and self.lidar_value_front_right > distance:
            self.CURRENT_STATE = WFMachine.STATE['TURN_LEFT']

        elif self.lidar_value_front > distance and self.lidar_value_front_left < distance and self.lidar_value_front_right < distance:
            self.CURRENT_STATE = WFMachine.STATE['FIND_WALL']

        elif self.lidar_value_front > distance and self.lidar_value_front_left < distance and self.lidar_value_front_right > distance:
            self.CURRENT_STATE = WFMachine.STATE['FIND_WALL']

        elif self.lidar_value_front > distance and self.lidar_value_front_left > distance and self.lidar_value_front_right < distance:
            self.CURRENT_STATE = WFMachine.STATE['FOLLOW_WALL']

        elif self.lidar_value_front > distance and self.lidar_value_front_left > distance and self.lidar_value_front_right > distance:
            self.CURRENT_STATE = WFMachine.STATE['FIND_WALL']

        else:
            rospy.INFO('unknow State')
            
    def run(self):

        if self.CURRENT_STATE == WFMachine.STATE['FIND_WALL']:
            print('...FIND_WALL...')
            self.__find_wall()
        
        if self.CURRENT_STATE == WFMachine.STATE['TURN_LEFT']:
            print('...TURN_LEFT...')
            self.__turn_left()
        
        elif self.CURRENT_STATE == WFMachine.STATE['FOLLOW_WALL']:
            print('...FOLLOW_WALL...')
            self.__follow_wall()

        
def lidar_cb(msg, wf):
    wf.lidar_values = msg

if __name__ == '__main__':
    rospy.init_node('wall_follower_node')
    wall_follower = WFMachine()
    rospy.Subscriber('/lidar_values', Lidar, callback=lidar_cb, callback_args=wall_follower)

    while not rospy.is_shutdown():
        wall_follower.determine_state()
        wall_follower.run()