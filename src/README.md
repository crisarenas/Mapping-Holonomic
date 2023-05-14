# Code Explanation
This section provides a more detailed explanation of what we are running. It is also possible to access some launch files to see their functionality. However, if you want to delve into the code in more detail, please refer to the [software requirements](https://github.com/crisarenas/Mapping-Holonomic/blob/main/Software_Requirements.md) section.

## Mapping Process
The command for mapping is the following:

```
roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true base_type:=omni_base arm:=false end_effector:=false world:=pal_kitchen
```

The [tiago_mapping launch file](https://github.com/crisarenas/Mapping-Holonomic/blob/main/src/tiago_mapping.launch) sets several arguments with default values and launches three main nodes. The three nodes will be explained below, with a particular focus on the one responsible for mapping. The highlighted nodes are the ones that play a crucial role in the task.



| Launch File   |      Node      |   Function   |
|:------------- |:---------------|:-------------|
| tiago_mapping |tiago_gazebo| This node starts the Gazebo simulation and loads the URDF model of the TIAGo robot into the simulated world. It also starts the hardware controllers necessary for the robot to move.     |
|   | rviz     | This node starts RViz to display the generated map and the planned trajectory of the robot in real time.|
|   |[navigation](https://github.com/crisarenas/Mapping-Holonomic/blob/main/src/navigation.launch)| This launcher starts the tiago_2dnav package for 2D navigation with TIAGo. It runs several route planning nodes, such as a global planner (global_planner) and a local planner (base_local_planner). It also runs perception nodes, such as the SLAM node.|
| navigation       | pal_navigation_sm |   This package launches a script called navigation.sh with arguments to configure some parameters related to navigation.     |
|        | **``pal_map_manager``** |  This package launches a node called `map_server`. map_server is an essential node in SLAM resolution, as it publishes the content of the map on a topic to which many other nodes can subscribe.|
|        |**`tiago_2dnav`**|The tiago_2dnav package plays a crucial role in robot navigation. It launches the`` state_machine.launch`` file, which is a finite state machine (FSM) designed to handle different navigation scenarios, such as moving to a specific location, following a path or avoiding obstacles. TIAGo navigates autonomously by transitioning between the different states of the FSM.|
|        |**``move_base``**|This package launches a launch file called ``move_base.launch``, which runs the move_base node responsible for planning trajectories and executing robot motion. The move_base node uses inputs such as the robot's current position, the desired goal position, and information from sensors such as laser scans to plan a trajectory. It also takes into account obstacles in the environment and adjusts the trajectory in real-time to avoid collisions. Once a trajectory is planned, the move_base node sends velocity commands to the robot's controllers to execute the motion.|
|        | pal_vo_server |This package launches a node called vo_server that provides a visual and semantic representation of the robot's location in real-time.|
|        |**``highways_server``**|This package launches a node called highways_server that provides route planning information for robot navigation.|
|        | **``robot_pose``**| This package launches a node called robot_pose_publisher that publishes the robot's position and orientation in the world reference frame.|
|        | ira_laser_tools|This package launches a node called laserscan_multi_merger that combines laser scanner data into a single scanner for trajectory planning.|
|        | laser_filters|This package launches a node called scan_to_scan_filter_chain that applies a filter to the laser scanner to improve its accuracy.|




## Localization Process
The commands for mapping are the following:

```
$ roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=true world:=pal_kitchen base_type:=omni_base arm:=false end_effector:=false map:=$HOME/.pal/tiago_maps/configurations/tiago_kitchen

$ rosservice call /pal_map_manager/save_map "directory: 'tiago_kitchen'"

$ rosservice call /global_localization "{}"

$ rosservice call /move_base/clear_costmaps "{}"
```

``tiago_navigation.launch`` includes three other launch files. Once again, Gazebo and RViz are launched. Additionally, "navigation.launch" is launched, but in this time the "state" parameter is set up for localization instead of mapping.

# Key Concepts:

### Global Planner:
* It is responsible for planning the complete route of the robot from its current position to the desired destination.
* It uses information from the map and other sensors to generate a high-level path that the robot can follow.
* It is useful for structured and known environments, where routes can be planned in advance.

### Local Planner:
* It is responsible for planning the detailed trajectory of the robot in real-time as it moves towards the desired destination.
* It uses more detailed information from sensors such as laser and camera data to avoid obstacles and adjust the trajectory as necessary.
* It is useful for unstructured or unknown environments where quick and accurate decisions must be made in real-time.