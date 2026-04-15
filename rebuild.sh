cd ~/dh_robot_ws
colcon build --packages-select my_drone_description
source install/setup.sh
ros2 launch my_drone_description display.launch.py
