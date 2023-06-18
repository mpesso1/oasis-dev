cmake_minimum_required(VERSION 3.5)
project(paths)

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
find_package(tutorial_interfaces REQUIRED)
find_package(srv_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# add_library(pathplanner_lib SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/PlannerTemplates/PathPlanner.h)

add_library(path_lib STATIC 
${CMAKE_CURRENT_SOURCE_DIR}/lib/PathPlanners/goPath.cpp ${CMAKE_CURRENT_SOURCE_DIR}/lib/PathPlanners/goPath.h)

target_include_directories(path_lib PUBLIC 
${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(path_lib PUBLIC 
Eigen3::Eigen
)

add_executable(pathplanner src/pathplanner.cpp 
lib/PlannerTemplates/PathPlanner.h
lib/PlannerTemplates/_goPath.h
lib/PlannerTemplates/noPath.h
)


target_include_directories(pathplanner PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${tutorial_interfaces_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/PlannerTemplates
  ${srv_interfaces_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/PathPlanners
)


# link dependencies
target_link_libraries(pathplanner PUBLIC
 ${rclcpp_LIBRARIES} 
 ${nav_msgs_LIBRARIES}
 ${tutorial_interfaces_LIBRARIES}
 Eigen3::Eigen
 ${srv_interfaces_LIBRARIES}
 path_lib
 )

target_compile_features(pathplanner PRIVATE cxx_std_17)

ament_export_dependencies(rclcpp)

# install executable
install(TARGETS
  pathplanner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
