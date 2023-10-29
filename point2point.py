#!/usr/bin/env python
import rospy
import sys
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion as efq
##sys.argv[0] parameter count
##sys.argv[1] argv[2] ... paramers

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def subcribe():
    rospy.Subscriber("/odom", Odometry, update_pose_callback)

def get_distance(goal):
    return math.sqrt((x - goal[0])**2 + (y - goal[1])**2)

def get_to_goal_direct(goal):
    return math.atan2(goal[1] - y, goal[0] - x)

def standarlize_angle(angle):
    if angle < -math.pi:
        return angle + 2*math.pi
    if angle > math.pi:
        return angle - 2*math.pi
    return angle

def get_delta_theta(goal):
    return standarlize_angle(get_to_goal_direct(goal) - theta)

def point2point(x_goal, y_goal, angle=None):
    if(angle != None):
        x_virtual = x_goal - virtual_distance*math.cos(angle)
        y_virtual = y_goal - virtual_distance*math.sin(angle)
        point2point(x_virtual, y_virtual)
    
    goal = (x_goal, y_goal)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    while not rospy.is_shutdown():
        control_vector = Twist()
        #print(get_delta_theta(goal))
        if(get_distance(goal) > goal_error):
            control_vector.linear.x = constrain(Kp_v * get_distance(goal), -MAX_LIN_VELOCITY, MAX_LIN_VELOCITY)
            control_vector.angular.z = constrain(Kp_w * get_delta_theta(goal), -MAX_ANG_VELOCITY, MAX_ANG_VELOCITY)
            #control_vector.linear.x = Kp_v * get_distance(goal)
            #control_vector.angular.z = Kp_w * get_delta_theta(goal)
            pub.publish(control_vector)
            rate.sleep()
        else:
            pub.publish(control_vector)
            break


def update_pose_callback(msg):
    global x, y, theta
    x, y = msg.pose.pose.position.x, msg.pose.pose.position.y 
    roll, pitch, theta = efq([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w], 'sxyz')


CONTROL_FREQ = 10
Kp_v = 0.1
Kp_w = 2
x, y, theta = 0, 0, 0
virtual_distance = 0.5
goal_error = 0.001
MAX_LIN_VELOCITY = 0.22
MAX_ANG_VELOCITY = 2.84
if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot', anonymous=True)
        rate = rospy.Rate(CONTROL_FREQ)
        subcribe()
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 3:
            print("Not enough parameter")
            pass
        else:
            if len(argv) == 3:
                print("target (x, y):", argv[1], argv[2])
                point2point(float(argv[1]), float(argv[2]))
            else:
                print("target (x, y, theta):", argv[1], argv[2], argv[3])
                point2point(float(argv[1]), float(argv[2]), float(argv[3]))
    except rospy.ROSInterruptException:
        pass