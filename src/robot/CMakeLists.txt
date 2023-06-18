cmake_minimum_required(VERSION 3.5)
project(robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(robot 
src/robot.cpp 
lib/SimulatorTemplate/Simulator.hpp
lib/SimulatorTemplate/SkipPose.hpp
)

target_include_directories(robot PUBLIC
${rclcpp_INCLUDE_DIRS}
${nav_msgs_INCLUDE_DIRS}
${tf2_ros_INCLUDE_DIRS}
)

target_link_libraries(robot PUBLIC
${rclcpp_LIBRARIES}
${nav_msgs_LIBRARIES}
Eigen3::Eigen
${tf2_ros_LIBRARIES}
)

ament_export_dependencies(rclcpp)

install(TARGETS robot DESTINATION lib/${PROJECT_NAME})

ament_package()
