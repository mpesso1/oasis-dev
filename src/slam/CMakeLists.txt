cmake_minimum_required(VERSION 3.5)
project(slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# find_package(gtsam REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# include_directories(/usr/include/eigen3/Eigen)


#target_link_libraries(gtsam Eigen3::Eigen)

# Build the slam executable
add_executable(slam src/slam.cpp)

target_link_libraries(slam PUBLIC ${rclcpp_LIBRARIES} ${nav_msgs_LIBRARIES}
${catkin_LIBRARIES}  
stdc++fs -lgtsam -ltbb -lstdc++fs
Eigen3::Eigen
)

target_include_directories(slam PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_export_dependencies(rclcpp)

target_compile_features(slam PRIVATE cxx_std_17)

# Install the executable
install(TARGETS slam DESTINATION lib/${PROJECT_NAME})



ament_package()
