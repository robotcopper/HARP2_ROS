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
ros2 launch robot_bringup nav2_bringup.launch.py use_TopicBasedSystem_hardware_interface:=True
```

`use_TopicBasedSystem_hardware_interface:=True` implies that it is no longer Gazebo with `ros2_control` (gazebo_ros2_control/GazeboSystem) responsible for providing the joint state, but `topic_based_ros2_control/TopicBasedSystem`, and therefore, no simulation is launched in this case.

### Simulation

To launch the simulation:

```bash
ros2 launch robot_bringup nav2_bringup.launch.py use_TopicBasedSystem_hardware_interface:=False
```

With `use_TopicBasedSystem_hardware_interface:=False`, the simulation is launched.


<br>
<br>

---
<br>
<br>

- Launching AMCL with a specific map:
```
 ros2 launch nav2_bringup localization_launch.py map:=<path_to_yaml_file_of_your_map>
```
<br>
<br>

- Launching micro_ros_agent on the device ttyACM0:
```
 ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```
