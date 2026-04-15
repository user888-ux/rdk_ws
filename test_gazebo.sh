#!/bin/bash
# 调整缩放和位置的完整云台启动脚本

# 设置路径
WORKSPACE="/home/user-888/dh_robot_ws"
MESHES_PATH="$WORKSPACE/src/my_drone_description/meshes"

# 设置环境变量
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib/gazebo_plugins
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
export OGRE_RTT_MODE=Copy
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# 创建完整云台URDF，调整缩放和添加地面
cat > /tmp/full_gimbal_scaled.urdf << 'EOF'
<?xml version="1.0"?>
<robot name="full_gimbal_scaled">
  
  <!-- 材质定义 -->
  <material name="white_plastic">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  
  <material name="blue_plastic">
    <color rgba="0.0 0.0 1.0 1.0"/>
  </material>
  
  <material name="green_plastic">
    <color rgba="0.0 1.0 0.0 1.0"/>
  </material>
  
  <material name="red_plastic">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
  
  <!-- 基础链接 - 底座，增加缩放 -->
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <!-- 增加缩放比例，尝试不同的值 -->
        <mesh filename="file:///home/user-888/dh_robot_ws/src/my_drone_description/meshes/base_link.STL" scale="1.0 1.0 1.0"/>
      </geometry>
      <material name="white_plastic"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
  </link>
  
  <!-- 第一个连接臂，增加缩放 -->
  <link name="link0">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/user-888/dh_robot_ws/src/my_drone_description/meshes/link0.STL" scale="1.0 1.0 1.0"/>
      </geometry>
      <material name="blue_plastic"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.02"/>
      </geometry>
    </collision>
  </link>
  
  <!-- 第一个关节 -->
  <joint name="joint0" type="revolute">
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="3.0"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  
  <!-- 第二个连接臂，增加缩放 -->
  <link name="link1">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/user-888/dh_robot_ws/src/my_drone_description/meshes/link1.STL" scale="1.0 1.0 1.0"/>
      </geometry>
      <material name="green_plastic"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.02"/>
      </geometry>
    </collision>
  </link>
  
  <!-- 第二个关节 -->
  <joint name="joint1" type="revolute">
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <parent link="link0"/>
    <child link="link1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="3.0"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  
  <!-- 第三个连接臂，增加缩放 -->
  <link name="link2">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/user-888/dh_robot_ws/src/my_drone_description/meshes/link2.STL" scale="1.0 1.0 1.0"/>
      </geometry>
      <material name="red_plastic"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.02"/>
      </geometry>
    </collision>
  </link>
  
  <!-- 第三个关节 -->
  <joint name="joint2" type="revolute">
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="3.0"/>
    <dynamics damping="0.7" friction="0.0"/>
  </joint>
  
  <!-- Gazebo配置 -->
  <gazebo reference="base_link">
    <material>Gazebo/White</material>
    <transparency>0.0</transparency>
  </gazebo>
  
  <gazebo reference="link0">
    <material>Gazebo/Blue</material>
    <transparency>0.0</transparency>
  </gazebo>
  
  <gazebo reference="link1">
    <material>Gazebo/Green</material>
    <transparency>0.0</transparency>
  </gazebo>
  
  <gazebo reference="link2">
    <material>Gazebo/Red</material>
    <transparency>0.0</transparency>
  </gazebo>
</robot>
EOF

echo "=== 启动Gazebo服务器 ==="
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
SERVER_PID=$!
sleep 5

echo "=== 启动机器人状态发布器 ==="
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tmp/full_gimbal_scaled.urdf)" &
RSP_PID=$!
sleep 2

echo "=== 生成完整云台模型（放在地面上方）==="
# 将模型放在z=1.0米高度，确保在地面上
ros2 run gazebo_ros spawn_entity.py -entity full_gimbal_scaled -topic robot_description -x 0 -y 0 -z 1.0

echo "=== 启动Gazebo客户端 ==="
gzclient --verbose

echo "=== 清理 ==="
kill $SERVER_PID $RSP_PID 2>/dev/null
rm /tmp/full_gimbal_scaled.urdf