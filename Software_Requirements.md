# Software Requirements

## Ubuntu version
The version of Ubuntu used in this project is Ubuntu 18.04.6 LTS (Bionic Beaver), but other versions of Ubuntu can also be used. If possible, it is recommended to run Ubuntu on a computer with a native Linux installation. However, in this case, it is installed in a VirtualBox VM via the [Ubuntu image](https://releases.ubuntu.com/18.04/). For the VM, it is recommended to use the unattended installation and allocate at least 25 GB of available memory, as the installation of ROS and the TIAGo package will occupy approximately 15 GB.

## ROS 
After installing Ubuntu, the next step is to install ROS (Robot Operating System). The version compatible with Ubuntu 18 is ROS Melodic, and its installation guide can be found [here](http://wiki.ros.org/melodic/Installation/Ubuntu). Additionally, the project uses the ROS 3D simulator known as Gazebo to visualize the real state of the robot in space, and the 2D visualizer RViz, which allows the user to see the robot's perception of its environment. These simulators will be installed with the ``sudo apt install ros-melodic-desktop-full``version.



## TIAGo Packages
Next, the TIAGo packages for ROS **Melodic** must be installed. To do so, follow the instructions on [this website](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS).

Finally, add the following lines at the end of the ``.bashrc`` file. The path of this file is ``/home/<username>/.bashrc``. Open it up using your preferred editor.
```
source /opt/ros/melodic/setup.bash
source /home/<username>/<your_workspace_name>/src/setup.bash
export ROS_WORKSPACE=/home/<username>/<your_workspace_name>
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WORSPACE
```
