import rospy
from geometry_msgs.msg import *
from Turtlebot3drive.turtlebot3drive import standarlize_angle
import math
#Create circular trajectory 
class Trajectory:
    def __init__(self, origin, radius, period):
        '''
        origin is in (x, y) form
        period: the time needed to move around the full circle (s)
        '''

        f = 1/period 
        self.pose_f =  lambda t: Pose2D( x = origin[0] + radius*math.cos(2*math.pi * f * t), 
                         y = origin[1] + radius * math.sin(2*math.pi * f * t), 
                        theta = standarlize_angle(2*math.pi * f * t + math.pi / 2))
        self.period = period
        self.has_theta = True
    

