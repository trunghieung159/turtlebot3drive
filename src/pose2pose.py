#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import *
from turtlebot3_drive.turtlebot3_drive import Turtlebot3_drive, standardlize_angle 
## pose2pose robot drive script 
## RUN : rosrun turtlebot3drive pose2pose.py [x] [y] [theta] 
## [x] [y] [theta] is goal pose, theta is optional 

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 3:
            print("Not enough parameter")
            pass
        else:
            turtlebot3 = Turtlebot3_drive(10)
            ##wait the odometry
            rospy.sleep(1)
            if len(argv) == 3:
                print("target (x, y):", argv[1], argv[2])
                goal = Pose2D(float(argv[1]), float(argv[2]), None)
                has_theta = False
            else:
                theta = standardlize_angle(float(argv[3]))
                print("target (x, y, theta): ", argv[1], argv[2], theta)
                goal = Pose2D(float(argv[1]), float(argv[2]), theta)
                has_theta = True 
            turtlebot3.pose_to_pose(goal, has_theta, True)
            print("ok")
    except rospy.ROSInterruptException:
        pass
