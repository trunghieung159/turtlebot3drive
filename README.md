Drive turtlebot3 in 2D environment arpproach desired pose(x , y, theta)
# A. Theory 
Turtlebot3 burgers are controlled by linear velocity ($v_c$) and angular velocity ($w_c$) with following constrains:  
$v_{max} = 0.22$ (m/s)  
$w_{max} = 2.84$ (rad/s)  
All angle / angle difference is standarlized in range $[-\pi \text{ ; } \pi)$  
Sensor range:  
$ r = 0.4 $ (m/s)    
Parameters:  
$K_v = 5$  
$K_w = 8$  
$K_{t\rho} = 0.75$  
$K_{t\theta} = 0.5$  
The robot is considred as being at goal if satisfies conditions:  
$\rho < \rho_{e} = 0.05$  
$\Delta_{\theta} < \Delta_{\theta_{e}} = 0.1$  

## 1. Approach goal without obstacle from $\textbf{p}(x, y, \theta)$ to $p_g(x_g, y_g)$ (excluding $\theta$)  
  ```
  approach_goal(goal_pose, has_theta = False)
  ```
  Robot to goal vector: $\overrightarrow{\Rho}(\Rho_x, \Rho_y)$   
  Measure diffrences, use propotional control :  
  $\rho = || \overrightarrow{\Rho} || = ||(p_g - p)||$  
  $\theta_{mtg} = atan2(\rho_{y}, \rho_{x})$  
  $\Delta_{\theta} = \theta_{g} - \theta$  
  $v = K_v * || \rho ||$  
  $w = K_w *\Delta_{\theta}$  
  
  Apply robot's constrains:   
  ```
  velo_control(linear_velocity, angular_velocity)
  ```
  $k = min(\frac{v_{max}}{v}, \frac{w_{max}}{||w||}, 1)$  
  $v_c = k * v$  
  $w_c = k * w$  
## 2. Approach goal without obstacle from $\textbf{p}(x, y, \theta)$ to $p_g(x_g, y_g, \theta_g)$ (including $\theta$)  
  ```
  approach_goal(goal_pose, has_theta = True)
  ```
  ### * In case $\rho \geq 1:$  
  Use the same algorithm with section 1  
  ### * In case $\rho_{e} < \rho < 1:$  
  #### 1. Determine temporary goal $p_{tg}(x_{tg}, y_{tg})$:  
  ##### 1. Delta theta between goal pose direction and move to goal direction:  
  $\Delta_{\theta_{g}} = \theta_g - \theta_{mtg}$  
  ##### 2. Temporary goal to goal direction:   
  $\theta_{tgtg} = \theta_{mtg} + (1 - K_{t\theta}) * \Delta_{\theta_{g}}$
  ##### 3. Measure temporary goal: 
  $x_{tg} = x_{g} - K_{t\rho} * \rho * cos(\theta_{tgtg})$  
  $y_{tg} = y_{g} - K_{t\rho} * \rho * sin(\theta_{tgtg})$  
  #### 2. Apply algorithm on section 1 for temporary goal $p_{tg}(x_{tg}, y_{tg})$ 
  ```
  approach_goal(temporary_pose, has_theta = False)
  ```
  ### * In case  $\rho < \rho_{e}:$  
  $\delta_{\theta} = |\theta - \theta_g|$  
  $v_c = 0, w_c = K_w * \delta_{\theta}$


## 3. Approach goal avoid obstacle from $\textbf{p}(x, y, \theta)$ to $p_g(x_g, y_g)$ / $p_g(x_g, y_g, \theta_g)$ 
```
approach_goal_avoid_obstacle(goal_pose, has_theta)
```
### 1. Obstacle avoidance vector $\overrightarrow{V_{oa}}$
Nearest obstacle position: $p_o (x_o, y_o)$  
Robot to nearest obstacle vector: $\overrightarrow{\Rho_o} = 
p_o - p$  
Distance between nearest obstacle and robot: $d = ||\overrightarrow{\Rho_o}|| $   
$$\overrightarrow{V_{oa}}
=
\begin{cases}
- \frac{r - d}{r - v_{max}} * v_{max} * \hat{\Rho}  \text{ if } d \le r \\ 
\overrightarrow{0} \text{ if } d > r
\end{cases}$$ 
### 2. Move to goal vector $\overrightarrow{V_{mtg}}$  
$\overrightarrow{V_{mtg}} = \overrightarrow{\Rho} * min(\frac{v_{max}}{\rho}, 1)$
### 3. Algorithm
#### * In case obstacles detected, the goal is further than the closest obstacle and obstacle avoidance vector is in opposite direction with move to goal vector: $(d < r)$, $\rho > d - v_{max},$ $|\angle \overrightarrow{V_{mtg}} - \angle \overrightarrow{V_{oa}}| > 3.0$: 
##### 1. Determine a perpendicular vector with magnitude of the last velocity vector or 0.8 * d:  
$|\overrightarrow{V_p}| = min(||\overrightarrow{V_{l}}||, 0.8*d)$  
$$ \angle \overrightarrow{V_p} = 
\begin{cases}
\angle \overrightarrow{V_{mtg}} + \frac{\pi}{2} \text{ if }| \angle \overrightarrow{V_l} - (\angle \overrightarrow{V_{mtg}} + \frac{\pi}{2}) | < | \angle \overrightarrow{V_l} - (\angle \overrightarrow{V_{mtg}} - \frac{\pi}{2}) | \\ 
\angle \overrightarrow{V_{mtg}} - \frac{\pi}{2} \text{ otherwise}  
\end{cases} $$
##### 2. Virtual goal $p_{v}(x_{p_{v}}, y_{p_{v}})$
$p_v = p + \overrightarrow{V_p} * min(\frac{v_{max}}{||\overrightarrow{V_p}||} ,1)$  
##### 3. Apply algorithm in section 1 for virtual goal  
```
approach_goal(virtual_goal_pose, has_theta = False)

```
#### * In case obstacles detected, the goal is further than the closest obstacle and obstacle avoidance vector is not in opposite direction with move to goal vector ($d < r$, $\rho > d - v_{max}$, $|\angle \overrightarrow{V_{mtg}} - \angle \overrightarrow{V_{oa}}| \le 3.0$):
##### 1. Composition vector: 
$\overrightarrow{V} = \overrightarrow{V_{oa}} + \overrightarrow{V_{mtg}}$  
##### 2. Virtual goal $p_{v}(x_{p_{v}}, y_{p_{v}})$
$p_v = p + \overrightarrow{V} * min(\frac{v_{max}}{||\overrightarrow{V}||} ,1) $
##### 3. Apply algorithm in section 1 for virtual goal  
```
approach_goal(virtual_goal_pose, has_theta = False)
```
#### * In case obstacles detected, the goal is closer than the closest obstacle($d < r$, $\rho \le d - v_{max}$):  
Apply algorithm in section 2 for goal pose:  
```
approach_goal(goal_pose, has_theta)
```
# B. Run simuation
## 1. Installation 
  ### 1.Install ROS: 
    http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment 
  ### 2. Install simulation: 
    https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
## 2. Clone package
```bash
cd ~/catkin_ws/src
source ~/catkin_ws/devel/setup.bash
git clone https://github.com/trunghieung159/turtlebot3drive.git
```
## 3. Run 
### 3.1 Run Roscore
```bash
source ~/catkin_ws/devel/setup.bash
catkin_make
roscore
```

### 3.2 Run simulation 
New terminal
```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
```
For single robot non-obstacle environment
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
For single robot environment with obstacles
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
For multiple robots formation control in non-obstacle environment
```bash
roslaunch turtlebot3drive 3robots_formation_empty.launch
```
For multiple robots formation control in obstacles existing environment
```bash
roslaunch turtlebot3drive 3robots_formation_obstacles.launch
```
### 3.3 Run drive script 
New terminal  
```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
rosrun turtlebot3drive SCRIPT_FILE [PARAMETER1] [PARAMETER2] ... [PARAMETER_N]
```
***Provided SCRIPTs:***
#### 1. Pose to pose (with / without obstacles)
```bash
pose2pose.py [x] [y] [theta]
```  
([x y theta] is pose of the target ;theta is optional)
#### 2. Move by trajectory (no obstacle)
##### Circular radius
```bash  
move_by_trajectory.py [x] [y] [r] [T]
```  
([x y] is coordinate of origin of circular trajectory, r is radius,  
and T is the expected time for the turtlebot to finish the trajectory)
##### From trajectory file
 ```bash
move_by_trajectory.py [path/file_name.csv] [deltaT] [has_theta]
```
(path/file_name.csv is the path to file name while deltaT is time (in seconds) between two step)  
(two consecutive columns of .csv file), has_theta determines whether the trajectory including  
theta or not, csv file cointains 3 columns corresponding to [x y theta].  
#### 3. Formation drive (with / without obstacles)
```bash
rosrun turtlebot3drive formation_control.py [x] [y] [theta] 
```
(x, y, theta) is pose of leader's target,
theta is optional

  
   


