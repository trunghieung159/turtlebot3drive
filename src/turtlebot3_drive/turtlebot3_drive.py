import rospy
import math
import time
import copy
import rosnode
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe
from constants_and_functions.constants import *
from constants_and_functions.functions import *


class Turtlebot3_drive:
    def __init__(self, control_freq = 10, robot_name = '', node_name = 'turtlebot3', 
                 max_velocity_vector_norm = MAX_LIN_VELOCITY):
        self.control_freq = control_freq
        self.robot_name = robot_name
        self.max_velocity_norm = max_velocity_vector_norm
        self.robot_pose = Pose2D()
        self.goal_pose = Pose2D()
        self.obstacle_distance = float('inf')
        self.obstacle_direction = None 
        self.velocity_vector = Vector3(0, 0, 0)
        
        if not node_name in rosnode.get_node_names():
            rospy.init_node(node_name, anonymous=False)
        self.rate = rospy.Rate(control_freq)
        rospy.Subscriber(robot_name + "/odom", Odometry, self.__update_robot_pose_callback)
        self.velo_pub = rospy.Publisher(robot_name + '/cmd_vel', Twist, queue_size=100)
        rospy.Subscriber(robot_name + "/scan", LaserScan, self.__read_laser_sensor_callback)
    
    def __update_robot_pose_callback(self, msg):
        '''Update robot's pose'''
        self.robot_pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, 
                    efq([msg.pose.pose.orientation.x, 
                         msg.pose.pose.orientation.y, 
                         msg.pose.pose.orientation.z, 
                         msg.pose.pose.orientation.w], 
                         'sxyz')[2])
        
    def __read_laser_sensor_callback(self, msg):
        '''Read sensor and update the direction to nearest obstacle 
        in worldframe coordinate and its distance from turtlebot3'''
        obstacle_relative_direction = None 
        min_distance = sensor_range
        
        # 0 degree corresponds to theta direction
        for i in range(0, 360):
            if msg.ranges[i] < sensor_range and msg.ranges[i] > msg.range_min:
                if msg.ranges[i] < min_distance:
                    min_distance = msg.ranges[i] 
                    obstacle_relative_direction = i 
        if obstacle_relative_direction != None:
            self.obstacle_direction = standardlize_angle(self.robot_pose.theta + 
                                                        (obstacle_relative_direction * angle_increment)) 
            self.obstacle_distance = min_distance 
        else:
            self.obstacle_direction = None
            self.obstacle_distance = float('inf') 

    def move_by_trajectory(self, trajectory):
        '''Control robot by trajectory. (without obstacle)
        
        pose is a function map t -> (x, y, theta)
        '''
        #Move to starting point of trajectory
        pose_f = trajectory.pose_f
        # goal_pose = pose_f(0)
        # self.pose_to_pose(goal_pose, trajectory.has_theta)
        delta_t = 1 / self.control_freq
        interval, total_intervals = 0, int(trajectory.period / delta_t)
        while interval < total_intervals:
            goal_pose = pose_f(interval * delta_t)
            self.rate.sleep()
            self.approach_goal(goal_pose, trajectory.has_theta)
            interval += 1
        self.pose_to_pose(goal_pose, trajectory.has_theta)
        self.velo_control(0, 0)

    def pose_to_pose(self, goal_pose, has_theta = True, avoid_obstacle = False):
        '''Control robot move to goal_pose.
        
        goal_pose can either be in (x, y) or (x, y, theta) form
        '''
        while not self.is_at_goal(goal_pose, has_theta):
            if avoid_obstacle:
                self.approach_goal_avoid_obstacle(goal_pose, has_theta)
            else:
                self.approach_goal(goal_pose, has_theta)
            self.rate.sleep()    
        self.stop_control()

    def approach_goal_avoid_obstacle(self, goal_pose, has_theta):
        '''Determine and apply control signal for robot to approach goal and avoid obstacle.

        Goal_pose is in (x, y) form.
        Need to rerun function or reset velocity after function finishes
        '''
        obstacle_distance = self.obstacle_distance
        obstacle_direction = self.obstacle_direction
        ## no obstacle detected or goal is closer than the closest obstacle
        if obstacle_direction == None or obstacle_distance == float('inf') or \
        self.get_distance(goal_pose) < obstacle_distance - MAX_LIN_VELOCITY:
            self.approach_goal(goal_pose, has_theta)
        ## obstacles detected and goal is further than the closest obstacle
        else:
            velocity_vector = self.get_sum_vector(goal_pose, obstacle_distance, obstacle_direction)    
            virtual_goal = Pose2D(self.robot_pose.x + velocity_vector.x, 
                                  self.robot_pose.y + velocity_vector.y, 
                                  0)
            self.pid_control(virtual_goal)
            self.velocity_vector = velocity_vector
            

    def approach_goal(self, goal_pose, has_theta):
        '''Determine and apply control signal for robot to approach goal.

        Goal_pose can either be in (x, y) or (x, y, theta) form.
        Need to rerun function or reset velocity after function finishes
        '''
        ## pose excludes theta
        if(not has_theta):
            velocity_vector = self.get_move_to_goal_vector(goal_pose)
            self.pid_control(goal_pose)
        ## pose includes theta
        else:
            ## Self-rotate case:
            ## robot is very close to goal_pose(x, y) 
            ## robot's theta differs from goal_pose's theta  
            ## delta_theta fluctuates wildly, robot needs to self-rotate 
            if(self.get_distance(goal_pose) < linear_goal_error):
                self.velo_control(0, Kp_w * \
                                 standardlize_angle(goal_pose.theta - self.robot_pose.theta))
                velocity_vector =  Vector3(0, 0, 0)
            else:
                ## Near goal case: Move to temporary goal    
                if(self.get_distance(goal_pose) < 1):
                    goal_pose = self.get_temporary_goal(goal_pose)
                ## Drive to (temporary) goal 
                velocity_vector = self.get_move_to_goal_vector(goal_pose)
                self.pid_control(goal_pose)     
        self.velocity_vector = velocity_vector

    def stop_control(self):
        '''Set robot's linear and angular velocity to 0'''
        self.velo_control(0, 0)
        self.velocity_vector = Vector3(0, 0, 0)

    def pid_control(self, goal_pose):
        '''Use PID to deterimine velocity 
         and control robot to approach 2D goal_pose(x, y)
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
        if linear_velocity == 0:
            k_v = 1
        else: 
            k_v = self.max_velocity_norm / linear_velocity
        if angular_velocity == 0:
            k_w = 1
        else:
            k_w = MAX_ANG_VELOCITY / abs(angular_velocity)
        k = min(k_v, k_w, 1)
        control_vector = Twist()
        control_vector.linear.x = linear_velocity * k
        control_vector.angular.z = angular_velocity * k
        self.velo_pub.publish(control_vector)

    def get_sum_vector(self, goal_pose, obstacle_distance, obstacle_direction):
        '''Return velocity sum vector of move to goal, obstacle avoidance vector
        or perpendicular vector'''

        move_to_goal_vector = self.get_move_to_goal_vector(goal_pose)           
        avoid_obstacle_vector = self.measure_avoid_obstacle_vector(obstacle_direction, 
                                                                   obstacle_distance)
        mtg_vector_direction = math.atan2(move_to_goal_vector.y, 
                                          move_to_goal_vector.x)
        ao_vector_direction = math.atan2(avoid_obstacle_vector.y, 
                                         avoid_obstacle_vector.x)
        angle_difference = abs(standardlize_angle(ao_vector_direction -  
                                                  mtg_vector_direction)) 
        ## mtg vector and ao vector is in opposite direction
        ## move robot in the line which is perpendicular to the mtg axis
        if angle_difference > 3.0:
            delta_angle = standardlize_angle(math.atan2(self.velocity_vector.y , self.velocity_vector.x) -
                                             mtg_vector_direction) 
            if delta_angle > 0:
                vector_direction = standardlize_angle(mtg_vector_direction + math.pi/2)
            else:
                vector_direction = standardlize_angle(mtg_vector_direction - math.pi/2)
            if obstacle_distance > self.max_velocity_norm:
                vector_magnitude = get_norm_3D_vector(self.velocity_vector) 
            else:
                vector_magnitude = obstacle_distance * 0.8
            velocity_vector = Vector3(vector_magnitude * math.cos(vector_direction),
                                      vector_magnitude * math.sin(vector_direction),
                                      0)
        else:
            velocity_vector = Vector3(move_to_goal_vector.x + avoid_obstacle_vector.x, 
                                      move_to_goal_vector.y + avoid_obstacle_vector.y, 
                                      0)
            velocity_vector = get_constrained_vector(velocity_vector, 
                                                     min(self.obstacle_distance, 
                                                         self.max_velocity_norm))
            
        return velocity_vector


    def get_move_to_goal_vector(self, goal_pose):
        ''' Return move to goal vector  

        vector norm is constrained by max velocity norm 
        '''
        move_to_goal_vector = Vector3(goal_pose.x - self.robot_pose.x, 
                                      goal_pose.y - self.robot_pose.y, 
                                      0) 
        return get_constrained_vector(move_to_goal_vector, self.max_velocity_norm)
    
    def get_avoid_obstacle_vector(self):
        '''Get obstacle avoidance vector 
        '''
        obstacle_distance = self.obstacle_distance
        obstacle_direction = self.obstacle_direction
        if obstacle_direction == None  or obstacle_distance == float('inf'):
            return Vector3(0, 0, 0)
        else:
            return self.measure_avoid_obstacle_vector(obstacle_direction, 
                                                      obstacle_distance)
        
    def measure_avoid_obstacle_vector(self, obstacle_direction, obstacle_distance):
        '''Determine obstacle avoidance vector 
        by obstacle_direction and obstacle_distance
        '''
        max_norm = self.max_velocity_norm 
        magnitude =   ((sensor_range - obstacle_distance) /
                      (sensor_range -  max_norm)) * max_norm
        return Vector3(- magnitude * math.cos(obstacle_direction), 
                       - magnitude * math.sin(obstacle_direction), 0)


    def is_at_goal(self, goal_pose, has_theta):
        ''' Return true if robot is at goal. Return false otherwise
        '''
        if(not has_theta):
            return self.get_distance(goal_pose) < linear_goal_error
        else:
            return (self.get_distance(goal_pose) < linear_goal_error and \
            abs(standardlize_angle(self.robot_pose.theta - goal_pose.theta)) \
                 < angular_goal_error)

    def get_distance(self, pose):
        ''' Get distance between robot and pose(x, y).
        '''

        return math.sqrt((self.robot_pose.x - pose.x)**2 + (self.robot_pose.y - pose.y)**2)
    
    def get_base_to_goal_direct(self, goal_pose):
        ''' Return the angular value of vector point from robot to goal_pose(x, y) '''

        return math.atan2(goal_pose.y - self.robot_pose.y, goal_pose.x - self.robot_pose.x)
    
    def get_delta_theta(self, goal_pose):
        '''Return the diffence between base to goal direction and robot's current direction
        
        goal_pose is in (x, y) form.
        '''
        return standardlize_angle(self.get_base_to_goal_direct(goal_pose) - self.robot_pose.theta)   

    def get_temporary_goal(self, goal_pose):
        ''' Determine temporary goal position (x, y) for pose-to-pose driving(including theta)
        
        Temporary goal is determined when robot are within 1m distance from goal_pose.
        Temporary goal_pose directs robot to the desired direction when reaching the goal_pose.
        '''
        distance = self.get_distance(goal_pose)
        delta_theta = standardlize_angle(goal_pose.theta - 
                                         self.get_base_to_goal_direct(goal_pose))
        temporary_goal_to_goal_direction = standardlize_angle(self.robot_pose.theta +
                                                               (1- K_t_theta) * delta_theta)
        return Pose2D(goal_pose.x - K_t_rho * distance * math.cos(temporary_goal_to_goal_direction), 
                      goal_pose.y - K_t_rho * distance * math.sin(temporary_goal_to_goal_direction),
                      None)

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
     
def mul_vector(vector, k):
    '''Return k * vector'''
    return Vector3(k * vector.x, k * vector.y, k * vector.z)

def add_vector(vector1, vector2):
    '''Return vector1 + vector2'''
    return Vector3(vector1.x + vector2.x,
                   vector1.y + vector2.y,
                   vector1.z + vector2.z)

def get_constrained_vector(vector, max_norm):
    '''Return vector with the same direction with original vector 
    but is constrained by maximum norm'''
    norm = get_norm_3D_vector(vector)
    if norm > max_norm:
        k = norm / max_norm
        constrained_vector = Vector3(vector.x / k, vector.y / k, vector.z / k)
        return constrained_vector
    else:
        return vector
    
