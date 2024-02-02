#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import *
from turtlebot3_drive.formation_turtlebot3 import Formation_turtlebot3

##drive robots in V-shaped formation 
if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 3:
            print("Not enough parameter")
            pass
        else:
            turtlebot3_formation = Formation_turtlebot3(10, 3)
            ##wait the odometry 
            rospy.sleep(1)
            if len(argv) == 3: 
                target = Pose2D(float(argv[1]), float(argv[2]), 0)
                print("formation control,", "target: [x y]:", float(argv[1]), float(argv[2]))
                turtlebot3_formation.formation_drive(target, False)
            else: 
                target = Pose2D(float(argv[1]), float(argv[2]), float(argv[3]))
                print("formation control,", "target: [x y theta]:", float(argv[1]), float(argv[2]), float(argv[3]))
                turtlebot3_formation.formation_drive(target, True)
            print("ok")
    except rospy.ROSInterruptException:
        pass