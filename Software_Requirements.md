# Software Requirements
The basic configuration for running this project involves installing Ubuntu 18, ROS Melodic, and the TIAGo workspace, as detailed in this document. However, to save time and effort, an alternative option is to [download]() and install the pre-configured virtual machine that I have been using. It comes with all the necessary components already installed and ready to use. Password:

## Ubuntu version
The version of Ubuntu used in this project is Ubuntu 18.04.6 LTS (Bionic Beaver), but other versions of Ubuntu can also be used. If possible, it is recommended to run Ubuntu on a computer with a native Linux installation. However, in this case, it is installed in a VirtualBox VM via the [Ubuntu image](https://releases.ubuntu.com/18.04/). For the VM, it is recommended to use the unattended installation and allocate at least 25 GB of available memory, as the installation of ROS and the TIAGo package will occupy approximately 15 GB.

## ROS 
After installing Ubuntu, the next step is to install ROS (Robot Operating System). The version compatible with Ubuntu 18 is ROS Melodic, and its installation guide can be found [here](http://wiki.ros.org/melodic/Installation/Ubuntu). Additionally, the project uses the ROS 3D simulator known as Gazebo to visualize the real state of the robot in space, and the 2D visualizer RViz, which allows the user to see the robot's perception of its environment. These simulators will be installed with the ``sudo apt install ros-melodic-desktop-full``version.



## TIAGo Packages
Next, the TIAGo packages for ROS **Melodic** must be installed. To do so, follow the instructions on [this website](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS). Keep in mind that the catkin build process may take more than 10 minutes.

Lastly, add the following lines to the end of the ``.bashrc`` file. The file path is ``/home/<username>/.bashrc``. Open it using your preferred text editor.
```
source /opt/ros/melodic/setup.bash
source /home/<username>/<your_workspace_name>/src/setup.bash
export ROS_WORKSPACE=/home/<username>/<your_workspace_name>
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_WORSPACE
```
