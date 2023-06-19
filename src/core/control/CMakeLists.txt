cmake_minimum_required(VERSION 3.5)
project(control)

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
find_package(srv_interfaces REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tflite-dist/include/)
add_library(tensorflowlite SHARED IMPORTED)
set_property(TARGET tensorflowlite PROPERTY IMPORTED_LOCATION 
${CMAKE_CURRENT_SOURCE_DIR}/tflite-dist/libs/linux_x64/libtensorflowlite.so
)

# add libraries
add_library(controller_lib STATIC
lib/ControllerTemplates/Controller.hpp)

set_target_properties(controller_lib PROPERTIES LINKER_LANGUAGE CXX)


target_include_directories(controller_lib PUBLIC 
"${CMAKE_CURRENT_SOURCE_DIR}/lib/ControllerTemplates" 
${CMAKE_CURRENT_SOURCE_DIR}/tflite-dist/include/)
target_link_libraries(controller_lib Eigen3::Eigen tensorflowlite)

# add executable
add_executable(control 
src/control.cpp
lib/ControllerTemplates/Controller.hpp
lib/ControllerTemplates/TestController.hpp
lib/ControllerTemplates/MountainCarController_DeepQ.hpp
lib/ControllerTemplates/MemoryDynamics.hpp
lib/ControllerTemplates/PID.hpp
)

target_include_directories(control PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${srv_interfaces_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/ControllerTemplates
)

# link dependencies
target_link_libraries(control PUBLIC 
controller_lib 
${rclcpp_LIBRARIES}
${nav_msgs_LIBRARIES}
${srv_interfaces_LIBRARIES}
)


target_compile_features(control PRIVATE cxx_std_17)


ament_export_dependencies(rclcpp)

# install executable
install(TARGETS
  control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
