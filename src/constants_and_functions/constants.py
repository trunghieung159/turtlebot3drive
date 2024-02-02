import rospy
import math
from geometry_msgs.msg import *

Kp_v = 5
Kp_w = 8
K_t_rho = 0.75
K_t_theta = 0.5
linear_goal_error = 0.05
angular_goal_error = 0.1
sensor_range = 0.4

##one degree in radians
angle_increment = math.pi/180

MAX_LIN_VELOCITY = 0.22
MAX_ANG_VELOCITY = 2.84
BURGER_RADIUS = 0.18

