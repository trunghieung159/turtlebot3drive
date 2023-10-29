# Installation 
1. Install ROS: <br/>
  http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment <br/>
2. Install simulation: </br>
  https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
# Create package
```bash
cd ~/catkin_ws/src
source ~/catkin_ws/devel/setup.bash
git clone https://github.com/trunghieung159/turtlebot3drive.git
```
# Run
Runs Roscore
```bash
source ~/catkin_ws/devel/setup.bash
catkin_make
roscore
```

New terminal: Running simulation 
```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

New terminal: Drive robot
```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
rosrun turtlebot3drive point2point.py [x] [y] [theta]
```


