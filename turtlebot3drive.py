import rospy
import math
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion as efq

Kp_v = 2
Kp_w = 8
x, y, theta = 0, 0, 0
virtual_distance = 1
goal_error = 0.01
MAX_LIN_VELOCITY = 0.22
MAX_ANG_VELOCITY = 2.84

class Turtlebot3_drive:
    def __init__(self, control_freq = 10):
        self.x, self.y, self.theta = 0, 0, 0
        rospy.init_node('turtlebot3', anonymous=True)
        self.rate = rospy.Rate(control_freq)
        rospy.Subscriber("/odom", Odometry, self.update_pose_callback)
        self.velo_control = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.set_goal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.set_goal_callback)
    
    def update_pose_callback(self, msg):
        '''Update robot's pose'''

        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y 
        _, _, self.theta = efq([msg.pose.pose.orientation.x, 
                           msg.pose.pose.orientation.y, 
                           msg.pose.pose.orientation.z, 
                           msg.pose.pose.orientation.w], 
                           'sxyz')
        
    def set_goal_callback(self, msg):
        # TODO:
        frame_ID = msg.header.frame_id 


    def point2point(self, goal):
        '''Move turtle bot to goal.

        Goal can either be (x, y) or (x, y, theta)
        '''   
        if len(goal)  == 3:
            x_virtual = goal[0] - virtual_distance*math.cos(goal[2])
            y_virtual = goal[1] - virtual_distance*math.sin(goal[2])
            self.point2point((x_virtual, y_virtual))
    
        ##TO DO: publish goal
        ##self.set_goal.publish()

        while not rospy.is_shutdown():
            control_vector = Twist()
            if(self.get_distance(goal) > goal_error):
                control_vector.linear.x = self.__constrain(Kp_v * self.get_distance(goal), -MAX_LIN_VELOCITY, MAX_LIN_VELOCITY)
                control_vector.angular.z = self.__constrain(Kp_w * self.get_delta_theta(goal), -MAX_ANG_VELOCITY, MAX_ANG_VELOCITY)
                #control_vector.linear.x = Kp_v * get_distance(goal)
                #control_vector.angular.z = Kp_w * get_delta_theta(goal)
                self.velo_control.publish(control_vector)
                self.rate.sleep()
            else:
                self.velo_control.publish(control_vector)
                break

    def get_distance(self, goal):
        ''' Get distance between robot and goal(x, y).
        '''

        return math.sqrt((self.x - goal[0])**2 + (self.y - goal[1])**2)
    
    def get_to_goal_direct(self, goal):
        ''' Return the angle of vector point from robot to goal(x, y) '''

        return math.atan2(goal[1] - self.y, goal[0] - self.x)
    
    def __standarlize_angle(self, angle):
        '''Standarlize an angle in range (-pi pi)

        return is_already_standardlized, standarlized_angle'''

        if angle < -math.pi:
            return False, angle + 2*math.pi
        if angle > math.pi:
            return False, angle - 2*math.pi
        return True, angle
        
    def get_delta_theta(self, goal):
        '''Get the difference of current direction and move to goal direction.
        
        Goal is in (x, y) form.
        '''
        
        return self.__standarlize_angle(self.get_to_goal_direct(goal) - self.theta)[1]    

    def __constrain(self, val, min_val, max_val):
        '''Constrain velocity in [min_val, max_val]'''

        return min(max_val, max(min_val, val))
    

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 3:
            print("Not enough parameter")
            pass
        else:
            turtlebot3 = Turtlebot3_drive(10)
            if len(argv) == 3:
                print("target (x, y):", argv[1], argv[2])
                turtlebot3.point2point((float(argv[1]), float(argv[2])))
            else:
                print("target (x, y, theta):", argv[1], argv[2], argv[3])
                turtlebot3.point2point((float(argv[1]), float(argv[2]), float(argv[3])))
    except rospy.ROSInterruptException:
        pass