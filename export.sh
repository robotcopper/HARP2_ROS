cd ../..
colcon build --packages-select robot_description
source /usr/share/gazebo/setup.bash && source install/setup.bash
xacro src/HARP2/robot_description/urdf/robot.urdf.xacro > src/HARP2/robot_description/urdf/robot.urdf
gz sdf -p src/HARP2/robot_description/urdf/robot.urdf > src/HARP2/robot_description/urdf/robot.sdf
colcon build --packages-select robot_description
source /usr/share/gazebo/setup.bash && source install/setup.bash
