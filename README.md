# Mapping with TIAGo Omni

This repository presents a process of mapping, localization, and navigation in a previously unknown map in ROS. For this purpose, the TIAGo robot from PAL Robotics is used in its omni-directional wheel configuration.

The necessary software requirements for this project can be found [here](https://github.com/crisarenas/Mapping-TIAGo/blob/main/Software_Requirements.md).


<p align="center">
<img src="Images/Tiago_omni.jpg" alt="TIAGo Robot" width="70%">
</p>

## Exploration and Mapping
The first step consists of creating a map of the environment using the **two LIDARs (SICK TiM561)** available in TIAGo and the gmapping ROS package. 

First, open a terminal, navigate to the TIAGo workspace and run the following command:

```
source ./devel/setup.bash
```

To launch the Gazebo TIAGo simulation with the omni base configuration, run the following command in a terminal. This command also opens RViz to visualize the mapping process and removes the arm and end effector of the robot, which are not necessary for this project.

```
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true base_type:=omni_base arm:=false end_effector:=false world:=simple_office_with_people
```

Then, in a separate terminal, run the following command to control the robot using the keyboard arrows. 

```
rosrun key_teleop key_teleop.py
```
The mapping process on Gazebo (real state) can be seen on the right, and the mapping process in RViz (robot state) can be seen on the left:

<p align="center">
<img src="Images/mapping.gif" width="80%">
</p>


Once the map has been fully explored, we can save it by running the command below. In this case, I have chosen to name the map "office". The map will be saved in the path: "/home/user/.pal/tiago_maps/office/".

```
rosservice call /pal_map_manager/save_map "directory: 'tiago_kitchen'"
```
Now, all terminals can be stopped. The resulting map is shown below.
<p align="center">
<img src="Images/tiago_kitchen/kitchen_map.png" width="40%">
</p>

## Localization
Once we have a map we can use AMCL localisation to match laser scans with the map to provide reliable estimates of the robot pose in the map.

Alternatively, you can launch the simulation with the TIAGo OMNI with the base_type parameter set to omni_base, by default this parameter is set to pmb2 :

By default the map launched is $(env HOME)/.pal/tiago_maps/configurations/$(arg world) where world is by default small_office. The world parameter can be changed by pal_office or simple_office:
```
roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true world:=pal_kitchen base_type:=omni_base arm:=false end_effector:=false map:=$HOME/.pal/tiago_maps/configurations/tiago_kitchen
```


As can be seen in the pictures below the robot wakes up in a position of the world which does not correspond to the map origin, which is the pose where the localization system assumes the robot is and where particles are spread representing possible poses of the robot.

In order to localize the robot run the following instruction in the second console. Esto lo pone en un sitio random y hay que localizarse.
```
rosservice call /global_localization "{}"
```
This causes the amcl probabilistic localization system to spread particles all over the map as shown in the picture below.


Now a good way to help the particle filter to converge to the right pose estimate is to move the robot. A safe way to do so is to make the robot rotate about itself. You may use the left and right arrows of the key_teleop in order to do so.

```
rosrun key_teleop key_teleop.py
```

Now, it is preferable to clear the costmaps as it contains erroneous data due to the bad localization of the robot:
```
rosservice call /move_base/clear_costmaps "{}"
```

The costmap now only takes into account obstacles in the static map and the system is ready to perform navigation.

The result is shown in the following pictures, where invalid particles are removed as more laser scans can be matched with the map and the localization converges eventually to the correct pose.


<p align="center">
<img src="Images/localization.gif" width="80%">
</p>


## Navigation 
Kill the teleoperation node and use the 2D Nav Goal tool in RViz. Just indicate the robot final position and orientation and a global planner will compute a path.


First of all make sure to kill the key_teleop node. Otherwise the robot will not move autonomously as key_teleop takes higher priority.

In order to send the robot to another location of the map the button 2D Nav Goal can be pressed in rviz.


Then, by clicking and holding the left button of the mouse over the point of the map at which we want to send the robot a green arrow will appear. By dragging the mouse around the point the orientation will change. This arrow represents the target position and orientation that the robot needs to reach.


When releasing the mouse button the global planner will compute a path, which will appear as a blue line starting at the current position of the robot and ending with a red arrow representing the desired target orientation.

Then, the robot will start moving following the trajectory, which will automatically re-arrange when getting too close to unexpected obstacles. The pictures below show different snapshots of rviz and gazebo during the navigation toward the selected goal.


Finally, the robot stops when the target goal pose have been reached up to a user-defined tolerance.

Autonomous navigation with rviz and TIAGo OMNI
After launching the simulation for the TIAGo OMNI you would have the same interaction with the button 2D Nav Goal as with the pmb2 as shown above; however the local planner is different.


The robot will start moving following the global trajectory. Given the flexibility and the freedom that the TIAGo OMNI provides, an appropriate Local Planner has to be used in order to exploit all the degrees of freedom of the base and plan trajectories in the plane in each direction and orientation. For this purpose, a special configuration of the teb_local_planner has been used to follow the path of a Global Planner while avoiding obstacles in a flexible way, by considering the kinematics model of the omnidirectional base.

The pictures below show rviz during the navigation toward the selected goal.

Finally, the robot stops when the target goal pose has been reached up to a user-defined tolerance.



<p align="center">
<img src="Images/navigation.gif" width="70%">
</p>

# Highlights and Challenges
* Comparativa turtlebot.
* No poder teleoperar el modo omnidirecional por completo.
* 