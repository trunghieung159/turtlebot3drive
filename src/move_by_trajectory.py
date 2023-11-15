#!/usr/bin/env python
import rospy
import sys
from Turtlebot3drive.turtlebot3drive import Turtlebot3_drive 
from Trajectory_generator.trajectory_generator import Trajectory
if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) != 3 and len(argv) != 5:
            print("Not enough parameter")
            pass
        elif len(argv) == 3:
            trajectory = Trajectory(argv[1], float(argv[2]))
            print("Trajectory from file: ", argv[1])

        else:
            print("Circular trajectory origin: (", float(argv[1]), ", ", float(argv[2]), ")",
                  "r = ", float(argv[3]), ", ", "T = ", float(argv[4]))
            trajectory = Trajectory((float(argv[1]), float(argv[2])), float(argv[3]), float(argv[4]))
        
        turtlebot3 = Turtlebot3_drive(10)
        turtlebot3.move_by_trajectory(trajectory)
    except rospy.ROSInterruptException:
        pass
