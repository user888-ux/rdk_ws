cmake_minimum_required(VERSION 3.8)
project(dh_robot_pkg)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找ament_cmake构建工具
find_package(ament_cmake REQUIRED)

# 查找ROS 2包依赖
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
# 地平线官方依赖
find_package(ai_msgs REQUIRED)

# 查找系统依赖
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")
set(OpenCV_LIBRARIES 
    opencv_core 
    opencv_imgproc 
    opencv_imgcodecs
    opencv_calib3d
    # 如果用到 highgui、videoio 等，也加进来
    # opencv_highgui
    # opencv_videoio
)


# 创建可执行文件
add_executable(dh_robot_node
  # src/dh_kinematics.cpp
  src/robot_controller.cpp
  src/main_node.cpp
  src/serial_communicator.cpp
  src/stereo_system.cpp
)

include_directories(${OpenCV_INCLUDE_DIRS})

# 设置包含目录
target_include_directories(dh_robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 使用现代ament方式设置所有依赖
ament_target_dependencies(dh_robot_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  orocos_kdl
  tf2_ros
  tf2
  ai_msgs
)

# 链接非ament包（系统库）
target_link_libraries(dh_robot_node
  Boost::boost  # 使用现代Boost目标
  yaml-cpp
  ${OpenCV_LIBRARIES}
)

# 安装目标到lib/${PROJECT_NAME}
install(TARGETS dh_robot_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
  PATTERN "*.hpp"
  PATTERN "*.h"
)

# 安装启动文件（如果存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

# 安装配置文件（如果存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# 安装包运行时依赖
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.sh"
  PATTERN "*.bash"
)

# 导出包含目录信息
ament_export_include_directories(include)

# 导出库依赖信息
ament_export_libraries(dh_robot_node)

# 导出现代依赖方式
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  orocos_kdl
  tf2_ros
  tf2
  ai_msgs
)

# 导出目标（现代方式）
ament_export_targets(export_${PROJECT_NAME})

# 最终调用ament_package()
ament_package()