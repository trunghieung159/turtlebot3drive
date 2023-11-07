#!/usr/bin/env python
import rospy
import sys
from Turtlebot3drive.turtlebot3drive import Turtlebot3_drive 
if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 3:
            print("Not enough parameter")
            pass
        else:
            turtlebot3 = Turtlebot3_drive(10)
            if len(argv) == 3:
                print("target (x, y):", argv[1], argv[2])
                turtlebot3.point_to_point((float(argv[1]), float(argv[2])))
                print("ok")
            else:
                print("target (x, y, theta):", argv[1], argv[2], argv[3])
                turtlebot3.point_to_point((float(argv[1]), float(argv[2]), float(argv[3])))
                print("ok")
    except rospy.ROSInterruptException:
        pass
