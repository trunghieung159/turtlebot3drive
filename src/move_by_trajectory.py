#!/usr/bin/env python
import rospy
import sys
from Turtlebot3drive.turtlebot3drive import Turtlebot3_drive 
from Trajectory_generator.trajectory_generator import Trajectory
if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) < 5:
            print("Not enough parameter")
            pass
        else:
            turtlebot3 = Turtlebot3_drive(10)
            print("Circular trajectory origin: (", float(argv[1]), ", ", float(argv[2]), ")",
                  "r = ", float(argv[3]), ", ", "T = ", float(argv[4]))
        turtlebot3 = Turtlebot3_drive(10)
        trajectory = Trajectory((float(argv[1]), float(argv[2])), float(argv[3]), float(argv[4]))
        turtlebot3.move_by_trajectory(trajectory)
    except rospy.ROSInterruptException:
        pass
