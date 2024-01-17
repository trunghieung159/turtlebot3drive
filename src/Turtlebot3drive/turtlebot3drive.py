import rospy
import math
import time
import copy
from threading import Thread
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe


Kp_v = 2
Kp_w = 8
linear_goal_error = 0.01
angular_goal_error = 0.1
sensor_range = 0.4

MAX_LIN_VELOCITY = 0.22
MAX_ANG_VELOCITY = 2.84

##convert one degree to radians
angle_increment = math.pi/180

class Turtlebot3_drive:
    def __init__(self, control_freq = 10):
        self.control_freq = control_freq
        self.robot_pose = Pose2D()
        self.goal_pose = Pose2D()
        self.obstacle_distance = 0
        self.obstacle_angle = None 
        rospy.init_node('turtlebot3', anonymous=True)

        self.rate = rospy.Rate(control_freq)
        rospy.Subscriber("/odom", Odometry, self.__update_robot_pose_callback)
        self.velo_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        # self.set_goal_pub = rospy.Publisher("move_base_simple/goal_pose", PoseStamped, queue_size = 10)
        # rospy.Subscriber("move_base_simple/goal_pose", PoseStamped, self.__receive_goal_callback)
        rospy.Subscriber("/scan", LaserScan, self.__read_laser_sensor_callback)
        ## stablize the odometry
    
    def __update_robot_pose_callback(self, msg):
        '''Update robot's pose'''
        self.robot_pose.x, self.robot_pose.y = msg.pose.pose.position.x, msg.pose.pose.position.y 
        _, _, self.robot_pose.theta = efq([msg.pose.pose.orientation.x, 
                           msg.pose.pose.orientation.y, 
                           msg.pose.pose.orientation.z, 
                           msg.pose.pose.orientation.w], 
                           'sxyz')
                
    def __read_laser_sensor_callback(self, msg):
        '''Read sensor and return the nearest obstacle's angle and distance
        in turtlebot's coordinate'''
        obstacle_angle = None 
        min_distance = sensor_range
        
        # 0 degree is equivalent to theta direction
        for i in range(0, 360):
            if msg.ranges[i] < sensor_range and msg.ranges[i] > msg.range_min:
                if msg.ranges[i] < min_distance:
                    min_distance = msg.ranges[i] 
                    obstacle_angle = i 
        self.obstacle_angle = obstacle_angle
        self.obstacle_distance = min_distance 
            

    def move_by_trajectory(self, trajectory):
        '''Control robot by trajectory.
        
        pose is a function map t -> (x, y, theta)
        '''
        #Move to starting point of trajectory
        pose_f = trajectory.pose_f
        # goal_pose = pose_f(0)
        # self.point_to_point(goal_pose, trajectory.has_theta)
        delta_t = 1 / self.control_freq
        interval, total_intervals = 0, int(trajectory.period / delta_t)
        while interval < total_intervals:
            goal_pose = pose_f(interval * delta_t)
            self.rate.sleep()
            self.approach_goal(goal_pose, trajectory.has_theta)
            interval += 1
        self.point_to_point(goal_pose, trajectory.has_theta)
        self.velo_control(0, 0)

    def point_to_point(self, goal_pose, has_theta = True, avoid_obstacle = False):
        '''Control robot move to goal_pose.
        
        goal_pose can either be in (x, y) or (x, y, theta) form
        '''
        if avoid_obstacle:
            has_theta = False 
        while not self.is_at_goal(goal_pose, has_theta):
            if avoid_obstacle:
                self.approach_goal_with_obstacle(goal_pose)
            else:
                self.approach_goal(goal_pose, has_theta)
            self.rate.sleep()    
            
        #Stop control        
        self.velo_control(0, 0)

    def approach_goal_with_obstacle(self, goal_pose):
        '''Determine and control robot to approach goal_pose and avoid obstacle.
        
        goal_pose is in (x, y) form,
        need to rerun function or reset velocity after function finishes
        '''
        move_to_goal_vector = self.get_move_to_goal_vector(goal_pose)
        ## has obstacle
        if self.obstacle_angle != None:
            obstacle_to_base_vector = Vector3(-self.obstacle_distance * math.cos(self.robot_pose.theta + angle_increment * self.obstacle_angle),
                    -self.obstacle_distance * math.sin(self.robot_pose.theta + angle_increment * self.obstacle_angle), 0)        
            ## current pose is closer to goal than cloest obstacle
            if get_norm_3D_vector(move_to_goal_vector) < self.obstacle_distance:
                velocity_vector = move_to_goal_vector
            else:
                mtg_vector_direction = math.atan2(move_to_goal_vector.y, move_to_goal_vector.x)
                otb_vector_direction = math.atan2(obstacle_to_base_vector.y, obstacle_to_base_vector.x)
                angle_difference = abs(standardlize_angle(otb_vector_direction - mtg_vector_direction)) 
                ## mtg vector and otp vector is in opposite direction
                if angle_difference > 3.1:
                    ## use a perpendicular vector as velocity vector
                        otp_vector_norm = get_norm_3D_vector(obstacle_to_base_vector)
                        velocity_vector = Vector3(otp_vector_norm / 2 * math.cos((otb_vector_direction + math.pi / 2)),
                                        otp_vector_norm / 2 * math.sin((otb_vector_direction + math.pi / 2)), 0)
                else:
                    avoid_obstacle_vector = self.__get_avoid_obstacle_vector(
                        self.obstacle_distance, angle_increment, self.obstacle_angle, self.robot_pose.theta)
                    velocity_vector = Vector3(move_to_goal_vector.x + avoid_obstacle_vector.x, move_to_goal_vector.y 
                                              + avoid_obstacle_vector.y, 0)
        ## no obstacle
        else:
            velocity_vector = move_to_goal_vector
        virtual_goal = Pose2D(self.robot_pose.x + velocity_vector.x, self.robot_pose.y + velocity_vector.y, 0)
        self.pid_control(virtual_goal)

    def approach_goal(self, goal_pose, has_theta):
        '''Determine and control robot to approach goal_pose without obstacle.
        
        goal_pose can either be in (x, y) or (x, y, theta) form,
        need to rerun function or reset velocity after function finishes
        '''
        ## pose excludes theta
        if(not has_theta):
            self.pid_control(goal_pose)
        ## pose includes theta
        else:
            ## Self-rotate case
            ## (unable to determine delta_theta) (robot is at the target(x, y) 
            ## but with different theta initially)
            if(self.get_distance(goal_pose) < linear_goal_error):
                self.velo_control(0, Kp_w*(goal_pose.theta - self.robot_pose.theta))

            ## Near goal: Move to virtual goal    
            elif(self.get_distance(goal_pose) < 1):
                goal_pose = self.get_virtual_goal(goal_pose)
                self.pid_control(goal_pose)        
            ## Far from goal: Move direct to goal
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

    def get_move_to_goal_vector(self, goal_pose):
        ''' Return move to goal vector  

        vector norm is constrained by max linear velocity of turtlebot3 
        '''
        distance = self.get_distance(goal_pose)
        max_norm = MAX_LIN_VELOCITY
        _, distance = self.__get_constrain(distance, -max_norm, max_norm)
        direction = self.get_base_to_goal_direct(goal_pose)
        x = math.cos(direction) * distance 
        y = math.sin(direction) * distance
        return Vector3(x, y, 0)

    def is_at_goal(self, goal_pose, has_theta):
        ''' Return true if robot is at goal. Return false otherwise
        '''
        if(not has_theta):
            return self.get_distance(goal_pose) < linear_goal_error
        else:
            return (self.get_distance(goal_pose) < linear_goal_error and 
            abs(self.robot_pose.theta - goal_pose.theta) < angular_goal_error)

    def get_distance(self, pose):
        ''' Get distance between robot and pose(x, y).
        '''

        return math.sqrt((self.robot_pose.x - pose.x)**2 + (self.robot_pose.y - pose.y)**2)
    
    def get_base_to_goal_direct(self, goal_pose):
        ''' Return the angular value of vector point from robot to goal_pose(x, y) '''

        return math.atan2(goal_pose.y - self.robot_pose.y, goal_pose.x - self.robot_pose.x)
    
    def get_delta_theta(self, goal_pose):
        '''Get the angle between robot's current direction and move-to-goal_pose direction.
        
        goal_pose is in (x, y) form.
        '''

        return standardlize_angle(self.get_base_to_goal_direct(goal_pose) - self.robot_pose.theta)   

    def get_virtual_goal(self, goal_pose):
        ''' return (x, y) of virtual goal_pose
        
        Virtual goal_pose is temporary goal_pose when robot are within 1m distance from goal_pose.
        Virtual goal_pose directs robot to the desired direction after reaching the goal_pose.
        '''
        distance = self.get_distance(goal_pose)
        angel_between = (self.get_base_to_goal_direct(goal_pose) + goal_pose.theta)/2
        if(abs(self.get_base_to_goal_direct(goal_pose) - goal_pose.theta) > math.pi):
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
    
    def __get_avoid_obstacle_vector(self, distance, angle_increment, angle, theta):
        '''Determine obstacle avoidance vector of an angle by messsage
        
        '''
        magnitude =  ((sensor_range - distance) / (sensor_range - MAX_LIN_VELOCITY)) * MAX_LIN_VELOCITY
        # magnitude = (1 / distance** 2) - (1 / sensor_range ** 2)
        if magnitude > MAX_LIN_VELOCITY:
            magnitude = MAX_LIN_VELOCITY
        return Vector3(- magnitude * math.cos(angle_increment * angle + theta), 
                    - magnitude * math.sin(angle_increment * angle + theta), 0)
    
    def print_sensor(self):
        print(self.oa_vector)

def standardlize_angle(angle):
    '''Standarlize an angle in range [-pi pi)
    return standardlized_angle'''
    standardlized_angle =  math.remainder(angle, math.tau)
    if standardlized_angle == math.pi:
        standardlized_angle = -standardlized_angle
    return standardlized_angle

def get_norm_3D_vector(vector):
    '''Return norm of 3D vector
    ''' 
    return math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)
     