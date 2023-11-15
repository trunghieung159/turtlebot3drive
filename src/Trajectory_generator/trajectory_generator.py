import rospy
from geometry_msgs.msg import *
from Turtlebot3drive.turtlebot3drive import standarlize_angle
import math
import pandas as pd
#Create circular trajectory 
class Trajectory:
    def __init__(self, origin, radius, period):
        '''
        origin is in (x, y) form
        period: the time needed to move around the full circle (s)
        '''

        f = 1/period 
        self.pose_f =  lambda t: Pose2D( x = origin[0] + radius * math.cos(2*math.pi * f * t), 
                         y = origin[1] + radius * math.sin(2*math.pi * f * t), 
                        theta = standarlize_angle(2*math.pi * f * t + math.pi / 2))
        self.period = period
        self.has_theta = True

    def __init__(self, file, delta_t):
        path = pd.read_csv(file, header=None)
        total_intervals = path.shape[0]
        self.pose_f =  lambda t: Pose2D(x = path.iloc[int(t / delta_t), 0], 
                         y = path.iloc[int(t / delta_t), 1], 
                        theta = None)
        self.period = delta_t * total_intervals
        self.has_theta = False
