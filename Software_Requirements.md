# Software Requirements
The basic configuration for running this project involves installing Ubuntu 18, ROS Melodic, and the TIAGo workspace, as detailed in this document. However two more alternatives are proposed: using a Virtual Machine or the raw code. My recommendation is to install everything from scratch following the next steps or to use the virtual machines. Opting to use the raw code can result in different problems that may be hard to resolve without previous experience.

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

___________
# Virtual Machine
To save time and effort, an alternative option is to [download](https://mega.nz/file/O2gU3bRI#fN3LesuJlF6LwxjBxxoQb-cPALUXt1mcRQgpwsuOCiU) and install the pre-configured virtual machine that I have been using. This virtual machine already includes all the necessary components, making it ready to use.

To install the virtual machine, follow these steps:

1. Download VirtualBox from the official website and install it on your system.
2. Visit [this web](https://www.alphr.com/ova-virtualbox/) and refer to the "Using OVA Files with VirtualBox" section for detailed instructions.
3. Importing the virtual machine may take more than an hour, so please be patient during the process.
If prompted for a password, use "admin" as the password.


_________

# Raw Code
The last option, which is not recommended for individuals with limited experience with ROS, is to directly use the ROS workspace and the React project. Both can be accessed through the following links. 

In the case of the ROS source code, it needs to be implemented in a workspace. As for React, the project is complete except for the node modules, which will be installed when running npm install.

* src folder inside tiago_ws: [download](https://drive.google.com/file/d/1TI9dlAwHL55C3hVILlkZrvx2NWBe95F-/view?usp=share_link)
* React project: [download.](https://drive.google.com/file/d/1lxwsD3TYcBRF-jBVHjWbFsRBJJGqoX9p/view?usp=share_link)


