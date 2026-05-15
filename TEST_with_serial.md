# 测试方法
cd ~/rdk_ws
colcon build
source install/setup.sh

# 以下终端的打开顺序 不能 调换

# 终端1
# 创建虚拟串口对（在后台运行）
socat -d -d pty,raw,echo=0 pty,raw,echo=0 &
# 然后应该能看到：
user-888@DESKTOP-1UA53FE:~$ 2025/10/28 17:47:58 socat[12280] N PTY is /dev/pts/9
2025/10/28 17:47:58 socat[12280] N PTY is /dev/pts/10
2025/10/28 17:47:58 socat[12280] N starting data transfer loop with FDs [5,5] and [7,7]

# 终端2
# 配置输出的串口为 /dev/pts/9或/dev/pts/10，配置波特率serial_baudrat
ros2 run dh_robot_pkg dh_robot_node --ros-args -p port:=/dev/pts/9 -p baud:=9600
# 然后应该能看到初始化信息：
[INFO] [1776682393.593019211] [robot_controller]: Successfully connected to serial port: /dev/pts/9
[INFO] [1776682393.593111423] [robot_controller]: Starting D-H Robot Controller

# 终端3：用python脚本批量发送数据
python3 ~/rdk_ws/serial_test.py