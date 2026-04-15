import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 只启动Gazebo服务器（无GUI）
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', 'worlds/empty.world'],
        output='screen',
        env={
            'GAZEBO_MODEL_PATH': os.path.expanduser('~/dh_robot_ws/install/my_drone_description/share'),
            'GAZEBO_RESOURCE_PATH': '/usr/share/gazebo-11'
        }
    )
    
    # 发布机器人状态
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': '''<robot name="test">
                <link name="base_link">
                    <visual>
                        <geometry><box size="0.3 0.3 0.3"/></geometry>
                        <material><color rgba="1 0 0 1"/></material>
                    </visual>
                </link>
            </robot>'''},
            {'use_sim_time': True}
        ]
    )
    
    # 延迟生成模型
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'test_drone',
                     '-topic', '/robot_description',
                     '-x', '0', '-y', '0', '-z', '1.0'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_server,
        robot_state_pub,
        spawn_entity# 
    ])
