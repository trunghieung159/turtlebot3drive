Drive turtlebot3
# 1. Installation 
  ## 1.Install ROS: 
    http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment 
  ## 2. Install simulation: 
    https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
# 2. Create package
```bash
cd ~/catkin_ws/src
source ~/catkin_ws/devel/setup.bash
git clone https://github.com/trunghieung159/turtlebot3drive.git
```
# 3. Run 
## 3.1 Run Roscore
```bash
source ~/catkin_ws/devel/setup.bash
catkin_make
roscore
```

## 3.2 Run simulation 
New terminal
```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
```
For non-obstacle environment
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
For environment with obstacles
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
## 3.3 Run drive script 
New terminal  
```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
rosrun turtlebot3drive SCRIPT_FILE [PARAMETER1] [PARAMETER2] [PARAMETER3]
```
***Provided SCRIPTs:***
### 1. Point to point (no obstacle) 
```bash
point2point.py [x] [y] [theta] False
```  
([x y theta] is pose of the target ;theta is optional)  
### 2. Move by trajectory (no obstacle)
#### Circular radius
```bash  
move_by_trajectory.py [x] [y] [r] [T]
```  
([x y] is coordinate of origin of circular trajectory, r is radius,  
and T is the expected time for the turtlebot to finish the trajectory)
#### From trajectory file
 ```bash
move_by_trajectory.py [path/file_name.csv] [deltaT] [has_theta]
```
(path/file_name.csv is the path to file name while deltaT is time (in seconds) between two step  
(two consecutive columns of .csv file), has_theta determines whether the trajectory including  
theta or not, csv file cointains 3 columns corresponding to [x y theta].  
### 3. Point to point (with obstacles)
```bash
rosrun turtlebot3drive point2point.py [x] [y] True
```
(x, y) is pose of target

  
   


