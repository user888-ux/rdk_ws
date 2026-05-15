# cmake_minimum_required(VERSION 3.8)
# project(dh_robot_pkg)

# # 设置C++标准
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# # 查找依赖包
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(orocos_kdl REQUIRED)

# # 查找 yaml-cpp
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(YAML_CPP yaml-cpp)

# # 创建运动学库
# add_library(dh_kinematics
#   src/dh_kinematics.cpp
#   src/robot_model.cpp
# )

# # 使用 ament_target_dependencies 设置依赖
# ament_target_dependencies(dh_kinematics
#   rclcpp
#   orocos_kdl
# )

# # 如果有 yaml-cpp，手动添加包含目录和链接
# if(YAML_CPP_FOUND)
#   target_include_directories(dh_kinematics PUBLIC ${YAML_CPP_INCLUDE_DIRS})
#   target_link_libraries(dh_kinematics ${YAML_CPP_LIBRARIES})
#   target_compile_definitions(dh_kinematics PUBLIC ${YAML_CPP_CFLAGS_OTHER})
# endif()

# # 创建机器人控制器库
# add_library(robot_controller
#   src/robot_controller.cpp
# )

# # 使用 ament_target_dependencies 设置依赖
# ament_target_dependencies(robot_controller
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   std_msgs
# )

# # 链接内部库
# target_link_libraries(robot_controller
#   dh_kinematics
# )

# # 创建主节点可执行文件
# add_executable(dh_robot_node src/main_node.cpp)

# # 使用 ament_target_dependencies 设置依赖
# ament_target_dependencies(dh_robot_node
#   rclcpp
# )

# # 链接内部库
# target_link_libraries(dh_robot_node
#   robot_controller
# )

# # 安装目标
# install(TARGETS
#   dh_robot_node
#   robot_controller
#   dh_kinematics
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # 安装头文件
# install(DIRECTORY include/
#   DESTINATION include
# )

# # 安装启动文件
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/
# )

# # 安装配置文件
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/
# )

# # 导出依赖
# ament_export_dependencies(
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   std_msgs
#   orocos_kdl
# )

# # 导出库
# ament_export_libraries(
#   dh_kinematics
#   robot_controller
# )

# ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(dh_robot_pkg)

# # 设置C++标准
# set(CMAKE_CXX_STANDARD 17)

# # 查找依赖包
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(orocos_kdl REQUIRED)
# find_package(Boost REQUIRED)
# find_package(yaml-cpp REQUIRED)

# # 包含目录
# include_directories(
#   include
#   ${Boost_INCLUDE_DIRS}
# )

# # 创建单个可执行文件，包含所有源文件
# add_executable(dh_robot_node
#   src/dh_kinematics.cpp
#   src/robot_model.cpp
#   src/robot_controller.cpp
#   src/main_node.cpp
# #  src/ros_to_serial.cpp
# #  src/serial_to_ros.cpp
# )

# # 设置包含目录
# target_include_directories(dh_robot_node PUBLIC
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# # 使用 ament_target_dependencies 设置所有依赖
# ament_target_dependencies(dh_robot_node
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   std_msgs
#   orocos_kdl
#   #serial
# )

# # 链接Boost库
# target_link_libraries(dh_robot_node
#   ${Boost_LIBRARIES}
# )

# # 手动链接 yaml-cpp
# target_link_libraries(dh_robot_node
#   yaml-cpp
# )

# # target_link_libraries(ros_to_serial ${rclcpp_LIBRARIES} serial::serial)
# # target_link_libraries(serial_to_ros ${rclcpp_LIBRARIES} serial::serial)

# # 安装目标
# install(TARGETS dh_robot_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# # 安装头文件
# install(DIRECTORY include/
#   DESTINATION include
# )

# # 安装启动文件
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/
# )

# # 安装配置文件
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/
# )

# # 导出依赖
# ament_export_dependencies(
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   std_msgs
#   orocos_kdl
#   #serial
# )

# ament_package()

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

# 查找系统依赖
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

# 创建可执行文件
add_executable(dh_robot_node
  src/dh_kinematics.cpp
  src/robot_controller.cpp
  src/main_node.cpp
  src/serial_communicator.cpp
)

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
)

# 链接非ament包（系统库）
target_link_libraries(dh_robot_node
  Boost::boost  # 使用现代Boost目标
  yaml-cpp
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
)

# 导出目标（现代方式）
ament_export_targets(export_${PROJECT_NAME})

# 最终调用ament_package()
ament_package()