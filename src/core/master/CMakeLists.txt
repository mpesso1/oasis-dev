cmake_minimum_required(VERSION 3.5)
project(master)

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
find_package(srv_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(master 
src/master.cpp 
lib/MasterTemplates/MasterTemplate.hpp
lib/Masters/WaypointReader.hpp
)

target_include_directories(master PUBLIC 
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include>"
${rclcpp_INCLUDE_DIRS}
${nav_msgs_INCLUDE_DIRS}
${srv_interfaces_INCLUDE_DIRS}
${std_msgs_INCLUDE_DIRS}
)

target_link_libraries(master PUBLIC 
${rclcpp_LIBRARIES}
${nav_msgs_LIBRARIES}
${srv_interfaces_LIBRARIES}
${std_msgs_LIBRARIES}
)

target_compile_features(master PRIVATE cxx_std_17)
ament_export_dependencies(rclcpp)

install(TARGETS
  master
  DESTINATION lib/${PROJECT_NAME})

ament_package()
