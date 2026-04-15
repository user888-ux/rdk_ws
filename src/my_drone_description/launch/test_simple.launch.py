# #!/usr/bin/env python3
# # 分步启动版launch文件

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('my_drone_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'drone.urdf')
    meshes_path = os.path.join(pkg_path, 'meshes')
    
    # 关键环境变量
    env_vars = {
        'GAZEBO_RESOURCE_PATH': f'/usr/share/gazebo-11:/usr/share/gazebo_plugins:{pkg_path}',
        'GAZEBO_MODEL_PATH': f'/usr/share/gazebo-11/models:/usr/share/gazebo_plugins/models:{pkg_path}/meshes:{pkg_path}/urdf:{meshes_path}',
        'GAZEBO_PLUGIN_PATH': '/opt/ros/humble/lib/gazebo_plugins:/opt/ros/humble/lib',
        'LD_LIBRARY_PATH': f"/opt/ros/humble/lib:{os.environ.get('LD_LIBRARY_PATH', '')}",
        'OGRE_RTT_MODE': 'Copy',
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
    }
    
    # 设置环境变量
    set_env_actions = [
        SetEnvironmentVariable(key, value) for key, value in env_vars.items()
    ]
    
    # 加载URDF
    if not os.path.exists(urdf_path):
        print(f"错误: URDF文件未找到: {urdf_path}")
        return LaunchDescription()
    
    robot_description_content = Command(['cat ', urdf_path])
    robot_description = {'robot_description': robot_description_content}
    
    # 创建world文件
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
    
    # 1. 启动Gazebo服务器
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
    
    # 2. 启动机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 3. 启动关节状态发布器
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 4. 生成无人机模型
    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'my_drone',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen',
        shell=False
    )
    
    # 5. 启动Gazebo客户端
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        shell=False
    )
    
    return LaunchDescription(set_env_actions + [
        # 启动Gazebo服务器
        gazebo_server,
        # 延迟启动机器人状态发布器
        TimerAction(period=2.0, actions=[robot_state_publisher]),
        TimerAction(period=2.5, actions=[joint_state_publisher]),
        # 延迟生成模型
        TimerAction(period=5.0, actions=[spawn_entity]),
        # 延迟启动Gazebo客户端
        TimerAction(period=7.0, actions=[gazebo_client]),
    ])
