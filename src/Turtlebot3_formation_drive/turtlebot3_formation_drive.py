import rospy
import math
import time
import copy
import numpy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool 
from Turtlebot3drive.turtlebot3drive import *



class Turtlebot3_formation_drive():
    def __init__(self, control_freq = 10, number_of_robots = 3, alpha = math.pi/3, d = 1):
        self.leader_id = math.ceil(float(number_of_robots) / 2) 
        self.robots = numpy.empty(number_of_robots, dtype= Turtlebot3_drive)
        self.number_of_robots = number_of_robots
        self.control_freq = control_freq
        self.alpha = alpha 
        self.d = d 
        rospy.init_node('turtlebot3_formation_control', anonymous=False)
        self.rate = rospy.Rate(control_freq)     
        for i in range(number_of_robots):
            self.robots[i] = Turtlebot3_drive(10, '/turtlebot3_' + str(i + 1), 
                                              'turtlebot3_formation_control')
        self.leader = self.robots[self.leader_id - 1]
        
    def formation_drive(self, goal_pose, has_theta):
        '''Move robots toward goal while maintaining formation
        goal_pose is in (x, y) form
        '''
        follower_ids = list(range(1, self.number_of_robots + 1))
        follower_ids.pop(self.leader_id - 1)
        ## run until leader reached the goal
        self.__drive_leader_to_goal(goal_pose, has_theta, follower_ids)
        ## wait followers reached the goal        
        self.__drive_followers_to_goal(has_theta, follower_ids)

    def get_formation_position(self, robot_id, has_theta):
        '''Get desired position for follwers based on leader's pose'''
        leader_pose = self.leader.robot_pose
        diffrence = robot_id - 2
        if diffrence < 0:
            follower_to_leader_formation_direction = standardlize_angle(leader_pose.theta - self.alpha / 2)
        else:
            follower_to_leader_formation_direction = standardlize_angle(leader_pose.theta + self.alpha / 2)
        if has_theta:
            theta = self.leader.robot_pose.theta
        else:
            theta = 0
        return Pose2D(leader_pose.x - self.d * abs(diffrence) * math.cos(follower_to_leader_formation_direction), 
                    leader_pose.y - self.d * abs(diffrence) * math.sin(follower_to_leader_formation_direction),
                    theta)

    
    def __drive_leader_to_goal(self, goal_pose, has_theta, follower_ids):
        '''Drive leader and followers until leader reaching the goal'''
        while not self.leader.is_at_goal(goal_pose, has_theta):
            leader_velocity_vector = self.leader.approach_goal_avoid_obstacle(goal_pose, has_theta)
            for i in follower_ids: 
                formation_position = self.get_formation_position(i, False)
                velocity_vector = self.__get_follower_velocity_vector(self.robots[i-1],
                                                                      formation_position,
                                                                      leader_velocity_vector)
                virtual_goal = Pose2D(self.robots[i-1].robot_pose.x + velocity_vector.x, 
                                    self.robots[i-1].robot_pose.y + velocity_vector.y, 0)
                self.robots[i-1].approach_goal_avoid_obstacle(virtual_goal, False)
            self.rate.sleep()
        self.leader.velo_control(0, 0)
    
    def __drive_followers_to_goal(self, has_theta, follower_ids):
        '''Drive followers to goals after leader reached goal'''
        follower_poses = numpy.empty(self.number_of_robots, dtype=Pose2D)
        reached_goal = numpy.empty(self.number_of_robots, dtype=bool)
        for i in follower_ids:
            follower_poses[i-1] = self.get_formation_position(i, True)
            reached_goal[i-1] = False
        reached_goal[self.leader_id - 1] = True 
        number_of_robots_reached_goal = 1
        while number_of_robots_reached_goal < self.number_of_robots:
            for i in follower_ids:
                if reached_goal[i-1]:
                    continue 
                if self.robots[i-1].is_at_goal(follower_poses[i-1], True):
                    self.robots[i-1].velo_control(0, 0)
                    reached_goal[i-1] = True 
                    number_of_robots_reached_goal += 1
                    continue 
                
                velocity_vector = self.__get_follower_velocity_vector(self.robots[i-1],
                                                        follower_poses[i-1], None)
                virtual_goal = Pose2D(self.robots[i-1].robot_pose.x + velocity_vector.x, 
                                    self.robots[i-1].robot_pose.y + velocity_vector.y, 
                                    self.leader.robot_pose.theta)
                self.robots[i-1].approach_goal_avoid_obstacle(virtual_goal, has_theta)
            self.rate.sleep()

    def __get_follower_velocity_vector(self, robot, formation_position, leader_velocity_vector):
        '''Determine velocity vector for a follower'''
        if leader_velocity_vector == None:
            leader_velocity_vector = Vector3(0, 0, 0)
        mf_vector = robot.get_move_to_goal_vector(formation_position)
        #mf_vector = Vector3(mf_vector.x / 2 , mf_vector.y / 2, 0)
        obstacle_avoidance_vector = robot.get_avoid_obstacle_vector()
        return Vector3(mf_vector.x + leader_velocity_vector.x + obstacle_avoidance_vector.x,
                                mf_vector.y + leader_velocity_vector.y + obstacle_avoidance_vector.y, 0)
  

    