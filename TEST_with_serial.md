# 测试方法
cd ~/dh_robot_ws
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
ros2 run dh_robot_pkg dh_robot_node --ros-args -p serial_port:=/dev/pts/9 -p serial_baudrate:=115200
# 然后应该能看到初始化信息：
[INFO] [1761645343.648921909] [robot_controller]: Using SCARA robot model
[INFO] [1761645343.649058917] [robot_controller]: Successfully connected to serial port: /dev/pts/9
[INFO] [1761645343.649157480] [robot_controller]: Serial communication established
[INFO] [1761645343.650783937] [robot_controller]: Robot controller initialized successfully
[INFO] [1761645343.650844321] [robot_controller]: Starting D-H Robot Controller

# 终端3：监听
ros2 topic echo /end_effector_pose

# 终端4：向输出串口发送信息
echo "0.5,0.3,-0.05" > /dev/pts/10
# 然后看到终端3：
user-888@DESKTOP-1UA53FE:~$ ros2 topic echo /end_effector_pose
header:
  stamp:
    sec: 1761645009
    nanosec: 842472623
  frame_id: base_link
pose:
  position:
    x: 0.17417667733679137
    y: 0.17933902272488067
    z: 0.25
  orientation:
    x: 0.0
    y: 0.0
    z: 0.36627252908604757
    w: 0.9305076219123143
---