#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import *
from Turtlebot3drive.turtlebot3drive import Turtlebot3_drive 
## point2point robot drive script 
## RUN (without obstacle) : rosrun turtlebot3drive point2point.py [x] [y] [theta] False
## theta is optional 
## RUN (with obstacle) : rosrun turtlebot3drive point2point.py [x] [y] True

if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 4:
            print("Not enough parameter")
            pass
        else:
            turtlebot3 = Turtlebot3_drive(10)
            rospy.sleep(1)
            #stablize the odometry
            if len(argv) == 4:
                avoid_obstacle = bool(argv[3])
                print("target (x, y):", argv[1], argv[2], "avoid obtacle =", avoid_obstacle)
                goal = Pose2D(float(argv[1]), float(argv[2]), None)
                has_theta = False
            else:
                avoid_obstacle = False
                print("target (x, y, theta): ", argv[1], argv[2], argv[3], ", avoid obstacle =", avoid_obstacle)
                goal = Pose2D(float(argv[1]), float(argv[2]), float(argv[3]))
                has_theta = True 
            turtlebot3.point_to_point(goal, has_theta, avoid_obstacle)
            print("ok")
    except rospy.ROSInterruptException:
        pass
