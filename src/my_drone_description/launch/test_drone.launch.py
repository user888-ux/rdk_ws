#!/usr/bin/env python3
# ROS 2 launch文件 - 加载外部URDF文件

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('my_drone_description')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_path, 'urdf', 'drone0.urdf')
    
    # 检查URDF文件是否存在
    if not os.path.exists(urdf_file):
        print(f"错误: URDF文件未找到: {urdf_file}")
        return LaunchDescription()
    
    # 设置环境变量
    env_vars = {
        'GAZEBO_RESOURCE_PATH': f'/usr/share/gazebo-11:/usr/share/gazebo_plugins:{pkg_path}',
        'GAZEBO_MODEL_PATH': f'/usr/share/gazebo-11/models:/usr/share/gazebo_plugins/models:{pkg_path}/meshes',
        'GAZEBO_PLUGIN_PATH': '/opt/ros/humble/lib/gazebo_plugins:/opt/ros/humble/lib',
        'LD_LIBRARY_PATH': f"/opt/ros/humble/lib:{os.environ.get('LD_LIBRARY_PATH', '')}",
        'OGRE_RTT_MODE': 'Copy',
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
    }
    
    # 创建环境变量设置动作
    set_env_actions = [
        SetEnvironmentVariable(key, value) for key, value in env_vars.items()
    ]
    
    # 创建world文件（如果不存在）
    worlds_dir = os.path.join(pkg_path, 'worlds')
    if not os.path.exists(worlds_dir):
        os.makedirs(worlds_dir)
    
    empty_world_path = os.path.join(worlds_dir, 'empty.world')
    if not os.path.exists(empty_world_path):
        with open(empty_world_path, 'w') as f:
            f.write('''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>''')
    
    # 启动Gazebo服务器
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            empty_world_path,
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen',
        shell=False
    )
    
    # 机器人状态发布器（从文件加载URDF）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )
    
    # 关节状态发布器
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )
    
    # 在Gazebo中生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_drone',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '100.0',  # 这里设置高度
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # 启动Gazebo客户端
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        shell=False
    )
    
    return LaunchDescription(set_env_actions + [
        # 先启动Gazebo服务器
        gazebo_server,
        # 启动机器人状态发布器
        robot_state_publisher,
        joint_state_publisher,
        # 延迟3秒后生成机器人（确保服务已启动）
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 3'],
            output='screen',
            shell=True
        ),
        spawn_entity,
        # 延迟5秒后启动Gazebo客户端
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 5'],
            output='screen',
            shell=True
        ),
        gazebo_client,
    ])