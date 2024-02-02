import rospy
from geometry_msgs.msg import *
from constants_and_functions.functions import standarlize_angle
import math
import pandas as pd
#Create circular trajectory 
class Trajectory:
    def __init__(self, origin = (0, 0), radius = 0, period = 0, file = None, delta_t = 0, has_theta = False):
        '''
        origin is in (x, y) form
        period: the time needed to move around the full circle (s)
        '''
        if file == None:
            f = 1/period 
            self.pose_f =  lambda t: Pose2D( x = origin[0] + radius * math.cos(2*math.pi * f * t), 
                            y = origin[1] + radius * math.sin(2*math.pi * f * t), 
                            theta = standarlize_angle(2*math.pi * f * t + math.pi / 2))
            self.period = period
            self.has_theta = True
        else:
            path = pd.read_csv(file, header=None)
            total_intervals = path.shape[0]
            self.pose_f =  lambda t: Pose2D(x = path.iloc[int(t / delta_t), 0], 
                            y = path.iloc[int(t / delta_t), 1], 
                            theta = None)
            self.period = delta_t * total_intervals
            self.has_theta = has_theta

        
