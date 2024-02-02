#!/usr/bin/env python
import rospy
import sys
from turtlebot3_drive.turtlebot3_drive import Turtlebot3_drive 
from trajectory_generator.trajectory_generator import Trajectory

## Drive turtlebot moves in a trajectory
## - Circular trajectory: rosrun move_by_trajectory.py [x] [y] [r] [T]
##      where [x y] is eulur coordinate of origin, r is radius and T is 
##      the expected time for the turtlebot to finish the trajectory.
## - Trajectory from csv file: rosrun move_by_trajectory.py [path/file_name.csv] [T] [hasTheta]
##      where T is the expected time for the turtlebot to finish the trajectory.
## Running examples:
##  rosrun turtlebot3drive move_by_trajectory.py 0 0 1 40
##  rosrun turtlebot3drive move_by_trajectory.py ~/catkin_ws/src/turtlebot3drive/src/path.csv 0.1 False
if __name__ == '__main__':
    try:
        argv = rospy.myargv(argv = sys.argv)
        if len(argv) != 4 and len(argv) != 5:
            print("Number of parameter must be 3 or 4")
            pass
        elif len(argv) == 4:
            trajectory = Trajectory(file=argv[1], delta_t=float(argv[2]), has_theta=bool(argv[3]))
            print("Trajectory from file: ", argv[1])
        else:
            print("Circular trajectory origin: (", float(argv[1]), ", ", float(argv[2]), "),",
                  "r =", float(argv[3]), ",", "T =", float(argv[4]))
            trajectory = Trajectory( origin=(float(argv[1]), float(argv[2])), radius=float(argv[3]), period=float(argv[4]))
        
        turtlebot3 = Turtlebot3_drive(10)
        turtlebot3.move_by_trajectory(trajectory)
    except rospy.ROSInterruptException:
        pass
