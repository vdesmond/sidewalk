cmake_minimum_required(VERSION 3.8)
project(sidewalk_cosim)

set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_node src/robot_node.cpp)
ament_target_dependencies(robot_node rclcpp geometry_msgs)

add_executable(bridge_node src/bridge_node.cpp)
ament_target_dependencies(bridge_node rclcpp geometry_msgs std_msgs)

install(TARGETS robot_node bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
