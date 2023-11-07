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
linear_goal_error = 0.01
angular_goal_error = 0.1
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

    def point_to_point(self, goal):
        '''Control robot move to goal.
        
        goal can either be in (x, y) or (x, y, theta) form
        '''
        if(len(goal) == 2):
            while self.get_distance(goal) > linear_goal_error:
                self.pid_point2point(goal)
                self.rate.sleep()
        else:
            mode = "far_from_goal"
            while (self.get_distance(goal) > linear_goal_error or 
                abs(self.theta - goal[2]) > angular_goal_error):
                if(self.get_distance(goal) < 1):
                    mode = "near_goal"

                #rotate case (robot is put closely to the goal at init time):
                if(self.get_distance(goal) < linear_goal_error):
                    while abs(self.theta - goal[2]) > angular_goal_error:
                        self.control_robot(0, Kp_w*(goal[2] - self.theta))
                        self.rate.sleep()
                    break

                if mode == "near_goal":
                    temporary_goal = self.get_virtual_goal(goal)
                else:
                    temporary_goal = goal 
                self.pid_point2point(temporary_goal)
                self.rate.sleep()
        #Stop control        
        self.control_robot(0, 0)
                
    def pid_point2point(self, goal):
        '''Use PID control to approach goal(x, y)
        without considering robot's angular at that target'''
        linear_velo = Kp_v * self.get_distance(goal)
        angular_velo = Kp_w * self.get_delta_theta(goal)
        self.control_robot(linear_velo, angular_velo)
        return 

    def control_robot(self, linear_velocity, angular_velocity):
        '''Control robot by linear velocity and angular velocity.
        
        If linear velocity or angular velocity is constrained by MAX_LIN_VELOCITY 
        or MAX_ANG_VELOCITY, the another velocity is decreased by the same factor
        to the keep R = linear_velocity / angular_velocity unchanged
        '''
        K_l, constrained_linear_velo = self.__get_constrain(linear_velocity, 
                                                                -MAX_LIN_VELOCITY, MAX_LIN_VELOCITY)
        K_w, constrained_angular_velo = self.__get_constrain(angular_velocity, 
                                                             -MAX_ANG_VELOCITY, MAX_ANG_VELOCITY)
    
        control_vector = control_vector = Twist()
        if K_l < K_w:
            control_vector.linear.x = constrained_linear_velo
            control_vector.angular.z = angular_velocity*K_l 
        elif K_w < K_l:
            control_vector.angular.z = constrained_angular_velo
            control_vector.linear.x = linear_velocity*K_w
        else:
            control_vector.linear.x = constrained_linear_velo
            control_vector.angular.z = constrained_angular_velo
        self.velo_control.publish(control_vector)


    def get_distance(self, goal):
        ''' Get distance between robot and goal(x, y).
        '''

        return math.sqrt((self.x - goal[0])**2 + (self.y - goal[1])**2)
    
    def get_to_goal_direct(self, goal):
        ''' Return the angular value of vector point from robot to goal(x, y) '''

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
        '''Get the angle between robot's current direction and move-to-goal direction.
        
        Goal is in (x, y) form.
        '''

        return self.__standarlize_angle(self.get_to_goal_direct(goal) - self.theta)[1]    

    def get_virtual_goal(self, goal):
        ''' return (x, y) of virtual goal
        
        Virtual goal is temporary goal when robot are within 1m distance from goal.
        Virtual goal directs robot to be at given direction after reaching the goal.
        '''
        distance = self.get_distance(goal)
        angel_between = (self.get_to_goal_direct(goal) + goal[2])/2
        if(abs(self.get_to_goal_direct(goal) - goal[2]) > math.pi):
            angel_between = angel_between + math.pi
        return (goal[0] - distance *3 / 4 * math.cos(angel_between), 
                goal[1] - distance *3 / 4 * math.sin(angel_between))

        


    def __get_constrain(self, val, min_val, max_val):
        '''Get constrained velocity in [min_val, max_val] of val
        
        Return [K, constrained_val] where K is constrained_val / val
        '''

        constrained_val =  min(max_val, max(min_val, val))
        if constrained_val == 0:
            return 1, 0
        else:
            return constrained_val / val, constrained_val
    


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
                turtlebot3.point_to_point((float(argv[1]), float(argv[2])))
                print("ok")
            else:
                print("target (x, y, theta):", argv[1], argv[2], argv[3])
                turtlebot3.point_to_point((float(argv[1]), float(argv[2]), float(argv[3])))
                print("ok")
    except rospy.ROSInterruptException:
        pass
