import rospy
import math
from geometry_msgs.msg import *
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
    