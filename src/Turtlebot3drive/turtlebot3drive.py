import rospy
import math
import time
from threading import Thread
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe

Kp_v = 2
Kp_w = 8
virtual_distance = 1
linear_goal_error = 0.01
angular_goal_error = 0.1
MAX_LIN_VELOCITY = 0.22
MAX_ANG_VELOCITY = 2.84

class Turtlebot3_drive:
    def __init__(self, control_freq = 10):
        self.control_freq = control_freq
        self.robot_pose = Pose2D()
        self.goal_pose = Pose2D()
        rospy.init_node('turtlebot3', anonymous=True)

        self.rate = rospy.Rate(control_freq)
        rospy.Subscriber("/odom", Odometry, self.update_robot_pose_callback)
        self.velo_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.set_goal_pub = rospy.Publisher("move_base_simple/goal_pose", PoseStamped, queue_size = 10)
        rospy.Subscriber("move_base_simple/goal_pose", PoseStamped, self.__receive_goal_callback)
        ## stablize the odometry
    
    def update_robot_pose_callback(self, msg):
        '''Update robot's pose'''
        self.robot_pose.x, self.robot_pose.y = msg.pose.pose.position.x, msg.pose.pose.position.y 
        _, _, self.robot_pose.theta = efq([msg.pose.pose.orientation.x, 
                           msg.pose.pose.orientation.y, 
                           msg.pose.pose.orientation.z, 
                           msg.pose.pose.orientation.w], 
                           'sxyz')
    
    def __receive_goal_callback(self, msg):
        frame_ID = msg.header.frame_id 
        _, _, self.goal_pose.theta  = efq([msg.pose.orientation.x, 
                           msg.pose.orientation.y, 
                           msg.pose.orientation.z, 
                           msg.pose.orientation.w], 
                           'sxyz')
        self.goal_pose.x = msg.pose.position.x
        self.goal_pose.y = msg.pose.position.y
        
    def __send_goal(self, trajectory):
        '''Update goal_pose  by trajectory regularly with time step delta_t = 1/f
        
        '''
        delta_t = 1 / self.control_freq
        interval = 1
        totalIntervals = trajectory.period / delta_t
        pose_f = trajectory.pose_f
        rate = rospy.Rate(self.control_freq)
        while interval <= totalIntervals:
            pose = pose_f(interval * delta_t)
            q = qfe(pose.x, pose.y, pose.theta)
            msg = PoseStamped()
            msg.pose.orientation.x,  msg.pose.orientation.y = q[0], q[1]
            msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]

            msg.pose.position = Point(pose.x, pose.y, 0)
            interval += 1
            self.set_goal_pub.publish(msg)
            rate.sleep()
            
            

    def move_by_trajectory(self, trajectory):
        '''Control robot by trajectory.
        
        pose is a function map t -> (x, y, theta)
        '''
        #Move to starting point of trajectory
        pose_f = trajectory.pose_f
        goal_pose = pose_f(0)
        self.point_to_point(goal_pose, trajectory.has_theta)
        delta_t = 1 / self.control_freq
        interval, total_intervals = 1, trajectory.period / delta_t
        while interval <= total_intervals:
            goal_pose = pose_f(interval * delta_t)
            self.rate.sleep()
            self.approach_goal(goal_pose, trajectory.has_theta)
            interval += 1
        self.point_to_point(goal_pose, trajectory.has_theta)
        self.velo_control(0, 0)

    def point_to_point(self, goal_pose, has_theta = True):
        '''Control robot move to goal_pose.
        
        goal_pose can either be in (x, y) or (x, y, theta) form
        '''
        while not self.is_at_goal(goal_pose, has_theta):
            self.approach_goal(goal_pose, has_theta)
            self.rate.sleep()    
        #Stop control        
        self.velo_control(0, 0)

    def approach_goal(self, goal_pose, has_theta = True):
        '''Determine and control robot to approach goal_pose.
        
        goal_pose can either be in (x, y) or (x, y, theta) form,
        need to rerun function or reset velocity after function finishes
        '''
        if(not has_theta):
            self.pid_control(goal_pose)
        else:
            ## Self-rotate case
            ## (unable to determine delta_theta) (robot is at the target(x, y) 
            ## but with different theta initially)
            if(self.get_distance(goal_pose) < linear_goal_error):
                self.velo_control(0, Kp_w*(goal_pose.theta - self.robot_pose.theta))

            ## Near goal case: Move to virtual goal    
            elif(self.get_distance(goal_pose) < 1):
                goal_pose = self.get_virtual_goal(goal_pose)
                self.pid_control(goal_pose)        
            ## Far from goal case: Move direct to goal
            else:
                self.pid_control(goal_pose)

    def pid_control(self, goal_pose):
        '''Use PID to deterimine velocity 
         and control robot to approach goal_pose(x, y)
        without considering robot's angular at that target
        
        need to rerun function or reset velocity after function finishes
        '''
        linear_velo = Kp_v * self.get_distance(goal_pose)
        angular_velo = Kp_w * self.get_delta_theta(goal_pose)
        self.velo_control(linear_velo, angular_velo)

    def velo_control(self, linear_velocity, angular_velocity):
        '''Apply constrain for linear velocity and angular velocity
        and control robot.
        
        If linear velocity or angular velocity is constrained by MAX_LIN_VELOCITY 
        or MAX_ANG_VELOCITY, the another velocity is decreased by the same factor
        to keep R = linear_velocity / angular_velocity unchanged. 
        '''
        K_l, constrained_linear_velo = self.__get_constrain(linear_velocity, 
                                                                -MAX_LIN_VELOCITY, MAX_LIN_VELOCITY)
        K_w, constrained_angular_velo = self.__get_constrain(angular_velocity, 
                                                             -MAX_ANG_VELOCITY, MAX_ANG_VELOCITY)
    
        control_vector = Twist()
        if K_l < K_w:
            control_vector.linear.x = constrained_linear_velo
            control_vector.angular.z = angular_velocity*K_l 
        elif K_w < K_l:
            control_vector.angular.z = constrained_angular_velo
            control_vector.linear.x = linear_velocity*K_w
        else:
            control_vector.linear.x = constrained_linear_velo
            control_vector.angular.z = constrained_angular_velo
        self.velo_pub.publish(control_vector)

    def is_at_goal(self, goal_pose, has_theta = True):
        if(not has_theta):
            return self.get_distance(goal_pose) < linear_goal_error
        else:
            return (self.get_distance(goal_pose) < linear_goal_error and 
            abs(self.robot_pose.theta - goal_pose.theta) < angular_goal_error)

    def get_distance(self, goal_pose):
        ''' Get distance between robot and goal_pose(x, y).
        '''

        return math.sqrt((self.robot_pose.x - goal_pose.x)**2 + (self.robot_pose.y - goal_pose.y)**2)
    
    def get_to_goal_direct(self, goal_pose):
        ''' Return the angular value of vector point from robot to goal_pose(x, y) '''

        return math.atan2(goal_pose.y - self.robot_pose.y, goal_pose.x - self.robot_pose.x)
    
    def get_delta_theta(self, goal_pose):
        '''Get the angle between robot's current direction and move-to-goal_pose direction.
        
        goal_pose is in (x, y) form.
        '''

        return standarlize_angle(self.get_to_goal_direct(goal_pose) - self.robot_pose.theta)   

    def get_virtual_goal(self, goal_pose):
        ''' return (x, y) of virtual goal_pose
        
        Virtual goal_pose is temporary goal_pose when robot are within 1m distance from goal_pose.
        Virtual goal_pose directs robot to the desired direction after reaching the goal_pose.
        '''
        distance = self.get_distance(goal_pose)
        angel_between = (self.get_to_goal_direct(goal_pose) + goal_pose.theta)/2
        if(abs(self.get_to_goal_direct(goal_pose) - goal_pose.theta) > math.pi):
            angel_between = angel_between + math.pi
        return Pose2D(goal_pose.x - distance *3 / 4 * math.cos(angel_between), 
                goal_pose.y - distance *3 / 4 * math.sin(angel_between),
                None)

    def __get_constrain(self, val, min_val, max_val):
        '''Get constrained velocity in [min_val, max_val] of val
        
        Return [K, constrained_val] where K is constrained_val / val
        '''
        if val == 0:
            return 1, 0
        constrained_val =  min(max_val, max(min_val, val))
        return constrained_val / val, constrained_val
    
def standarlize_angle(angle):
    '''Standarlize an angle in range [-pi pi)

    return standarlized_angle'''
    return math.atan2(math.sin(angle), math.cos(angle))
 
        