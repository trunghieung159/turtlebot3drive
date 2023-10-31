#!/usr/bin/env python
import rospy
import sys
import turtlebot3drive 
if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 3:
            print("Not enough parameter")
            pass
        else:
            turtlebot3 = turtlebot3drive.Turtlebot3_drive(10)
            if len(argv) == 3:
                print("target (x, y):", argv[1], argv[2])
                turtlebot3.point2point((float(argv[1]), float(argv[2])))
            else:
                print("target (x, y, theta):", argv[1], argv[2], argv[3])
                turtlebot3.point2point((float(argv[1]), float(argv[2]), float(argv[3])))
    except rospy.ROSInterruptException:
        pass

