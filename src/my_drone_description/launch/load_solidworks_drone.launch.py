import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # URDF文件完整路径
    urdf_file = "/home/user-888/dh_robot_ws/install/my_drone_description/share/my_drone_description/urdf/drone.urdf"
    
    # 一个命令完成所有操作
    combined_command = (
        'unset LD_LIBRARY_PATH && '
        'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib && '
        'export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11 && '
        'echo "启动Gazebo服务端..." && '
        'gzserver --verbose -s libgazebo_ros_factory.so & '
        'GZSERVER_PID=$! && '
        'sleep 3 && '
        'echo "发布URDF到参数服务器..." && '
        'ros2 run robot_state_publisher robot_state_publisher ' + urdf_file + ' & '
        'RSP_PID=$! && '
        'sleep 2 && '
        'echo "在Gazebo中生成无人机模型..." && '
        'ros2 run gazebo_ros spawn_entity.py -entity drone -file "' + urdf_file + '" -x 0 -y 0 -z 1.0 && '
        'echo "等待进程..." && '
        'wait $RSP_PID && '
        'kill $GZSERVER_PID'
    )
    
    gazebo_process = ExecuteProcess(
        cmd=['bash', '-c', combined_command],
        output='screen'
    )

    return LaunchDescription([
        gazebo_process,
    ])