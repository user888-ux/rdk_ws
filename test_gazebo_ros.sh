#!/bin/bash
# 保存为 test_gazebo_ros.sh

echo "=== 测试Gazebo ROS插件 ==="
echo ""

# 1. 启动Gazebo服务器（后台运行）
echo "1. 启动Gazebo服务器..."
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
SERVER_PID=$!
sleep 5

# 2. 检查服务是否可用
echo "2. 检查/spawn_entity服务..."
ros2 service list | grep spawn_entity
if [ $? -eq 0 ]; then
    echo "✓ /spawn_entity服务可用"
else
    echo "✗ /spawn_entity服务不可用"
    kill $SERVER_PID
    exit 1
fi

# 3. 启动Gazebo客户端
echo "3. 启动Gazebo客户端..."
gzclient &
CLIENT_PID=$!
sleep 3

# 4. 测试spawn服务
echo "4. 测试生成机器人..."
# 创建一个简单的URDF
cat > /tmp/test_box.urdf << 'EOF'
<?xml version="1.0"?>
<robot name="test_box">
  <link name="base_link">
    <visual>
      <geometry><box size="0.5 0.5 0.5"/></geometry>
      <material name="red"><color rgba="1 0 0 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.5 0.5 0.5"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
</robot>
EOF

# 发布URDF到参数服务器
ros2 param set /robot_state_publisher robot_description "$(cat /tmp/test_box.urdf)" &

# 生成机器人
ros2 run gazebo_ros spawn_entity.py -entity test_box -file /tmp/test_box.urdf -x 0 -y 0 -z 1.0

if [ $? -eq 0 ]; then
    echo "✓ 机器人生成成功"
else
    echo "✗ 机器人生成失败"
fi

echo ""
echo "测试完成，按Ctrl+C退出所有进程"
echo "Gazebo进程PID: $SERVER_PID, $CLIENT_PID"

# 等待用户中断
wait
