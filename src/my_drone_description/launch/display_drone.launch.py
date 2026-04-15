#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('my_drone_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'drone.urdf')
    
    # 检查URDF文件是否存在
    if not os.path.exists(urdf_path):
        print(f"错误: URDF文件未找到: {urdf_path}")
        print("请确保drone.urdf文件存在于my_drone_description包的urdf目录中")
    
    # 加载URDF到参数服务器
    robot_description_content = Command([
        'xacro ', urdf_path
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # 创建worlds目录和空的world文件
    worlds_dir = os.path.join(pkg_path, 'worlds')
    if not os.path.exists(worlds_dir):
        os.makedirs(worlds_dir)
    
    empty_world_path = os.path.join(worlds_dir, 'empty.world')
    if not os.path.exists(empty_world_path):
        with open(empty_world_path, 'w') as f:
            f.write('<?xml version="1.0"?>\n<sdf version="1.6">\n  <world name="default">\n    <include>\n      <uri>model://ground_plane</uri>\n    </include>\n    <include>\n      <uri>model://sun</uri>\n    </include>\n  </world>\n</sdf>')
    
    # 机器人状态发布器节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 关节状态发布器节点
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 创建所有节点和launch文件的列表
    nodes = [
        robot_state_publisher,
        joint_state_publisher,
    ]
    
    # 先启动Gazebo服务器，然后生成机器人
    # 使用ExecuteProcess直接启动Gazebo
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', empty_world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': f'{pkg_path}/meshes:{pkg_path}/urdf:{pkg_path}',
            'GAZEBO_RESOURCE_PATH': '/usr/share/gazebo-11:/usr/share/gazebo_plugins'
        }
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
            '-z', '0.5',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # Gazebo客户端（GUI）
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env={
            'GAZEBO_MODEL_PATH': f'{pkg_path}/meshes:{pkg_path}/urdf:{pkg_path}',
            'GAZEBO_RESOURCE_PATH': '/usr/share/gazebo-11:/usr/share/gazebo_plugins'
        }
    )
    
    return LaunchDescription([
        # 先启动Gazebo服务器
        gazebo_server,
        # 然后启动机器人状态发布器
        robot_state_publisher,
        joint_state_publisher,
        # 等待Gazebo服务器启动后生成机器人
        spawn_entity,
        # 最后启动Gazebo客户端（GUI）
        gazebo_client,
    ])