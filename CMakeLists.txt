cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

# Require C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(number_publisher_cpp src/number_publisher.cpp)
ament_target_dependencies(number_publisher_cpp rclcpp std_msgs)

add_executable(number_counter_cpp src/number_counter.cpp)
ament_target_dependencies(number_counter_cpp rclcpp std_msgs)

install(TARGETS
  number_publisher_cpp
  number_counter_cpp
  DESTINATION lib/${PROJECT_NAME})

ament_package()
