cmake_minimum_required(VERSION 3.8)
project(teraranger_evo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)  # Changed to find the ROS2 serial package

include_directories(include)

add_executable(teraranger_evo_node
  src/teraranger_evo_node.cpp
)

ament_target_dependencies(teraranger_evo_node
  rclcpp
  sensor_msgs
  serial  # Added serial as dependency
)

install(TARGETS
  teraranger_evo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()