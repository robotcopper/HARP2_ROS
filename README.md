# H.A.R.P

![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=plastic&logo=ubuntu&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-A22846?style=plastic&logo=Raspberry%20Pi&logoColor=white)
![Raspberry Pi Pico](https://img.shields.io/badge/Raspberry%20Pi%20Pico-008287?style=plastic&logo=Raspberry%20Pi&logoColor=white)
![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=plastic&logo=github&logoColor=white)

[![BSD 3-Clause License](https://img.shields.io/badge/License-BSD%203--Clause-0EA94B)](https://opensource.org/licenses/BSD-3-Clause)

![ROS 2](https://img.shields.io/badge/ROS2-humble-0EA94B?style=flat&logo=ros&logoColor=22314E)
![ROS 2](https://img.shields.io/badge/ROS2-humble-0EA94B?style=flat&logo=ros&logoColor=white&labelColor=22314E)

---

### Launching the Robot

### Real Robot

To launch the real robot (publishing motor commands on the `/topic_based_joint_commands` topic) by publishing velocity commands on the `/omnidirectional_controller/cmd_vel_unstamped` topic:

Navigation should remap `/cmd_vel` to this topic during deployment (requires real hardware: not tested yet).

```bash
ros2 launch robot_bringup robot_nav_bringup.launch.py use_TopicBasedSystem_hardware_interface:=True
```

`use_TopicBasedSystem_hardware_interface:=True` implies that it is no longer Gazebo with `ros2_control` (gazebo_ros2_control/GazeboSystem) responsible for providing the joint state, but `topic_based_ros2_control/TopicBasedSystem`, and therefore, no simulation is launched in this case.

### Simulation

To launch the simulation:

```bash
ros2 launch robot_bringup robot_nav_bringup.launch.py use_TopicBasedSystem_hardware_interface:=False
```

With `use_TopicBasedSystem_hardware_interface:=False`, the simulation is launched.


<br>
<br>

---
<br>
<br>

- Launching AMCL with a specific map:
```bash
 ros2 launch nav2_bringup localization_launch.py map:=<path_to_yaml_file_of_your_map>
```
<br>
<br>

- Launching micro_ros_agent on the device ttyACM0:
```bash
 ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

<br>


## Install

After cloning the repos in your workspace

```bash
cd ~/ros2_ws
rosdep init
rosdep update
sudo apt update

#install omnidirectional_controllers dependencies
rosdep install --from-paths src/HARP2/omnidirectional_controllers --ignore-src -r -y --rosdistro humble

#install YDLidar-SDK dependencies
mkdir ~/ros2_ws/src/HARP2/ydlidar/YDLidar-SDK/build
cd ~/ros2_ws/src/HARP2/ydlidar/YDLidar-SDK/build
cmake ..
make
sudo make install

#install xacro
sudo apt install ros-humble-xacro

#install joint-state-publisher
sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui

#install gazebo11
cd ~
git clone https://github.com/osrf/gazebo.git -b gazebo11
cd gazebo
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
#you may have to install third party dependencies

#install tf2-geometry-msgs
sudo apt install ros-humble-tf2-geometry-msgs
#install nav-msgs
sudo apt install ros-humble-nav-msgs
#install std-srvs
sudo apt install ros-humble-std-srvs

#install gazebo_ros_pkgs
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/ros-simulation/gazebo_ros_pkgs.git

#install gazebo_ros2_control
cd ~/ros2_ws/src
git clone -b humble git@github.com:ros-controls/gazebo_ros2_control.git

#install nav2
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

#install slam-toolbox
sudo apt install ros-humble-slam-toolbox

#install uros
mkdir ~/ros2_ws/src/uros
cd ~/ros2_ws/src/uros
git clone -b humble git@github.com:micro-ROS/micro-ROS-Agent.git
git clone -b humble git@github.com:micro-ROS/micro_ros_msgs.git

#install ros2_controll
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

```

<br>


## Important Notes on Deployment

- The lidar simulation in `robot_description/urdf/lidar/lidar.xacro` is in "gpu_ray" and needs to be modified to "ray" in the absence of gpu on the deployment hardware.

- The YDLidar X4 and the YDLidar TminiPlus share the same unique device attributes and therefore do not have a unique udev rule for the hardware and must depend on the USB port they are plugged into.