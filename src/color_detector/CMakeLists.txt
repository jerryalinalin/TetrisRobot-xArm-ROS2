cmake_minimum_required(VERSION 3.8)
project(color_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedTarget.msg"
  "msg/DetectedTargetArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Add compile-time DATA_DIR for Python script location
set(DATA_DIR_DEFINE "DATA_DIR=\"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\"")

# ===== detection_node =====
add_executable(detection_node
  src/detection_node.cpp
  src/ColorClassiFication.cpp
  src/target.cpp
)
target_include_directories(detection_node PRIVATE include)
target_compile_definitions(detection_node PRIVATE ${DATA_DIR_DEFINE})
ament_target_dependencies(detection_node
  rclcpp
  cv_bridge
  sensor_msgs
  std_msgs
)
target_link_libraries(detection_node
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
install(TARGETS detection_node DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(DIRECTORY script/ DESTINATION share/${PROJECT_NAME}/script)

ament_package()
