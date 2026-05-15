cmake_minimum_required(VERSION 3.8)
project(camera_detector)

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
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 地平线dnn库依赖
find_package(ai_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(hobot_mot REQUIRED)

# 查找系统依赖
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

# 创建可执行文件
add_executable(camera_detector_node
    src/main.cpp
    src/detector_controller.cpp
    src/yolov5_detector.cpp   # 你的检测器实现
)

# 设置包含目录
target_include_directories(camera_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 使用现代ament方式设置所有依赖
ament_target_dependencies(camera_detector_node
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
  orocos_kdl
  OpenCV
  # boost
  yaml-cpp
  # 地平线相关库
  dnn_node
  # ai_msgs
  hobot_mot
  hbm_img_msgs
)

# 链接非ament包（系统库）
target_link_libraries(camera_detector_node
  # Boost::boost  # 使用现代Boost目标
  yaml-cpp
)

# 安装目标到lib/${PROJECT_NAME}
install(TARGETS camera_detector_node
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
ament_export_libraries(camera_detector_node)

# 导出现代依赖方式
ament_export_dependencies(

)

# 导出目标（现代方式）
ament_export_targets(export_${PROJECT_NAME})

# 最终调用ament_package()
ament_package()