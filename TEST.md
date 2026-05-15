# 测试方法
cd ~/dh_robot_ws
colcon build
source install/setup.sh

# 以下终端的打开顺序 不能 调换

# 终端1：启动节点
ros2 run dh_robot_pkg dh_robot_node

# 终端2：查看末端位姿
ros2 topic echo /end_effector_pose

# 终端3：发布正确的关节状态（3个关节）
<!-- # sensor_msgs/msg/JointState 这是ROS中用于描述机器人所有关节状态的标准消息类型 -->
# !!! 发布的参数说明：
#    sec: 0        # 时间戳（秒）
#    nanosec: 0    # 时间戳（纳秒）
# 'base_link' 表示这些关节状态是相对于机器人基座坐标系的

# position:
# joint1: 位置 = 0.5 弧度（约28.65度）
# joint2: 位置 = 0.3 弧度（约17.19度）
# joint3: 位置 = -0.05 米（向下的位移）
# 注意：对于SCARA机器人：
# 关节1和2是旋转关节 → 单位是弧度
# 关节3是平移关节 → 单位是米

# velocity:
# 表示每个关节的瞬时速度：
# 所有关节速度都为0，表示机器人当前静止
# 单位：旋转关节是弧度/秒，平移关节是米/秒

# effort:
# 表示每个关节的输出力矩/力：
# 所有关节力矩都为0
# 单位：旋转关节是牛顿·米，平移关节是牛顿
ros2 topic pub /joint_states sensor_msgs/msg/JointState "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
name: ['joint1', 'joint2', 'joint3']
position: [0.5, 0.3, -0.05]
velocity: [0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0]
" --once