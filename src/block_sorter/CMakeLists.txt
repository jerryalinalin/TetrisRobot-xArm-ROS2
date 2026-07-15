cmake_minimum_required(VERSION 3.8)
project(block_sorter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(color_detector REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)


# Generate MoveOnce service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveOnce.srv"
  DEPENDENCIES builtin_interfaces
)

# Compile-time path to installed resources
set(DATA_DIR_DEFINE "DATA_DIR=\"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\"")

# ===== sorter_node: strategy + robot control =====
add_executable(sorter_node src/sorter_node.cpp)
target_include_directories(sorter_node PRIVATE include)
target_compile_definitions(sorter_node PRIVATE ${DATA_DIR_DEFINE})
ament_target_dependencies(sorter_node
  rclcpp
  std_msgs
  sensor_msgs
  color_detector
  xarm_msgs
  OpenCV
)
target_link_libraries(sorter_node
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
install(TARGETS sorter_node DESTINATION lib/${PROJECT_NAME})

# ===== move_service: pick-and-place service server =====
add_executable(move_service src/move_service.cpp)
target_include_directories(move_service PRIVATE include)
target_compile_definitions(move_service PRIVATE ${DATA_DIR_DEFINE})
ament_target_dependencies(move_service
  rclcpp
  std_msgs
  xarm_msgs
)
target_link_libraries(move_service
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
install(TARGETS move_service DESTINATION lib/${PROJECT_NAME})

# Install data files
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)

ament_package()
