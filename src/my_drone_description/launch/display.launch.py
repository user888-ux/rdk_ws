import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 设置Gazebo环境变量,确保Gazebo能找到它的资源文件
    set_gazebo_env = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=os.path.join(os.environ.get('HOME'), '.gazebo')
    )

    # 获取 gazebo_ros 包的路径（不是你的包！）
    gazebo_ros_pkg_share = get_package_share_directory('gazebo_ros')
    
    # 方法一：如果你的URDF在ROS2包中（推荐做法）
    pkg_share = FindPackageShare(package='my_drone_description').find('my_drone_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'drone.urdf')

    # 启动Gazebo服务端（必须加载ROS插件）
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )
    
    # 启动Gazebo客户端（GUI）
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # 读取URDF文件
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # 启动Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_pkg_share, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': '', 'verbose': 'true'}.items()
    )
    
    # 发布机器人描述到参数服务器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file],
        parameters=[{'use_sim_time': True}]  # 添加：使用仿真时间
    )
    
    # 将机器人生成到Gazebo中
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_drone', 
            '-topic', 'robot_description',  # 从参数服务器读取
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )
    
    # 启动RViz以可视化模型（可选）
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', '/opt/ros/humble/share/rviz_default_plugins/rviz/default.rviz'],
    #     output='screen'
    # )

    return LaunchDescription([
        # 设置环境变量
        set_gazebo_env,
        # 启动Gazebo
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        # rviz_node
    ])


# import os
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # 1. 核心路径配置
#     gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
#     my_drone_pkg = get_package_share_directory('my_drone_description')
#     urdf_path = os.path.join(my_drone_pkg, 'urdf', 'drone.urdf')

#     # 2. 读取URDF内容（ROS2标准写法）
#     with open(urdf_path, 'r') as f:
#         robot_description = f.read()

#     # 3. 启动Gazebo（适配WSL，强制软件渲染）
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
#         ),
#         launch_arguments={
#             'world': 'empty.world',
#             'verbose': 'true'
#         }.items()
#     )

#     # 4. 发布机器人描述（同步仿真时间）
#     robot_state_pub = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[
#             {'robot_description': robot_description},
#             {'use_sim_time': True}  # 关键：同步Gazebo仿真时间
#         ]
#     )

#     # 5. 延迟生成模型（Gazebo启动需要时间，WSL下延迟5秒更稳）
#     spawn_entity = TimerAction(
#         period=5.0,  # 延迟5秒，确保Gazebo完全启动
#         actions=[
#             Node(
#                 package='gazebo_ros',
#                 executable='spawn_entity.py',
#                 name='spawn_entity',
#                 arguments=[
#                     '-entity', 'my_drone',
#                     '-x', '0', '-y', '0', '-z', '0.5',
#                     '-topic', '/robot_description'
#                 ],
#                 output='screen'
#             )
#         ]
#     )

#     # 6. 组装启动项
#     return LaunchDescription([
#         gazebo,
#         robot_state_pub,
#         spawn_entity
#     ])