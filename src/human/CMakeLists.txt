cmake_minimum_required(VERSION 3.5)
project(human)

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
find_package(srv_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(human 
src/human.cpp
lib/KeyStrokes.hpp
)

target_include_directories(human PUBLIC 
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include>"
${rclcpp_INCLUDE_DIRS}
${srv_interfaces_INCLUDE_DIRS}
${CMAKE_CURRENT_SOURCE_DIR}/lib
${std_msgs_INCLUDE_DIRS}
)

target_link_libraries(human PUBLIC
${rclcpp_LIBRARIES}
${srv_interfaces_LIBRARIES}
ncurses
${std_msgs_LIBRARIES}
)

target_compile_features(human PRIVATE cxx_std_17)
ament_export_dependencies(rclcpp)

install(TARGETS
  human
  DESTINATION lib/${PROJECT_NAME})

ament_package()
