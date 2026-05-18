# dh_robot_pkg

让RDK X5与云台建立串口连接，并接受角度和解算，再返回结果

## robot_controller.cpp 管理各个发布者和订阅者
## serial_communicator.cpp 定义建立串口与传输的方式
## dh_angles.cpp 纯数学库，完成二维云台的姿态逆解
## main_node.cpp 主节点

#

# 使用方法
## 方法一（电脑端测试）

### 以下终端的打开顺序 不能 调换

### 终端1
### 创建虚拟串口对（在后台运行）
> ```bash
>socat -d -d pty,raw,echo=0 pty,raw,echo=0 &
### 然后应该能看到：
> ```txt
>user-888@DESKTOP-1UA53FE:~$ 2025/10/28 17:47:58 socat[12280] N PTY is /dev/pts/9
>2025/10/28 17:47:58 socat[12280] N PTY is /dev/pts/10
>2025/10/28 17:47:58 socat[12280] N starting data transfer loop with FDs [5,5] and [7,7]

### 终端2
> ```bash
>cd ~/rdk_ws
>colcon build
>source install/setup.sh
### 配置输出的串口为 /dev/pts/9或/dev/pts/10，配置波特率serial_baudrat
> ```bash
>ros2 run dh_robot_pkg dh_robot_node --ros-args -p port:=/dev/pts/9 -p baud:=9600
### 然后应该能看到初始化信息：
> ```txt
>[INFO] [1776682393.593019211] [robot_controller]: Successfully connected to serial port: /dev/pts/9
>[INFO] [1776682393.593111423] [robot_controller]: Starting D-H Robot Controller

### 终端3：用python脚本批量发送数据
它自动发送12字节（4+4+帧尾）的帧(我们约定好的数据格式)，监听节点马上收到信息，并解析出对应yaw和pitch角。
> ```bash
>python3 ~/rdk_ws/serial_test.py

<div align="center">
  <img src="./src/dh_robot_pkg/result.png" width="80%" alt="Inference Result"/>
</div>

### 可以看到成功显示yaw和pitch角并收到解算后的角度，经过16进制数转换，最终确认了从缓冲区中得到的数据和打印出来的转换后的角度能一一对应

关于鲁棒性，他和我们的搜索逻辑有关，目前的搜索逻辑是：
搜索帧尾，没搜到就保留缓冲区中的数据，搜到了就取离帧尾最前的8个字节，并清空缓冲区的数据，并转换成角度。
假如有以下几种情况：
- 1.帧尾被替换个别字节
- 2.帧尾丢失个别字节
- 3.数据部分被替换个别字节
- 4.数据部分丢失个别字节。
根据我们的搜索逻辑可知，1.2.4基本无影响，对于3，加入了-180~180的角度判断，把影响缩减了部分。

扩展部分：如果以后要修改发送帧的长度，比如增加角速度，那也不难改，只需要提取前12个字节即可。

## 方法二（板子上测试）

### 在 RDKX5 上用USB转串口模块连接一对串口，这里以 /dev/ttyS1 为例

### 终端1
> ```bash
>cd ~/rdk_ws
>colcon build
>source install/setup.sh
### 配置输出的串口为 /dev/ttyS1,配置波特率serial_baudrat
> ```bash
>ros2 run dh_robot_pkg dh_robot_node --ros-args -p port:=/dev/ttyS1 -p baud:=115200
### 然后应该能看到初始化信息：
> ```txt
>[INFO] [1776682393.593019211] [robot_controller]: Successfully connected to serial port: /dev/ttyS1
>[INFO] [1776682393.593111423] [robot_controller]: Starting D-H Robot Controller

### 终端2：用python脚本批量发送数据
> ```bash
>python3 ~/rdk_ws/serial_test.py
### 同上