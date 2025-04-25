cmake_minimum_required(VERSION 3.8)
project(motor_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find pigpio (look in /usr/local/lib)
find_library(PIGPIO_LIBRARY NAMES pigpio HINTS /usr/local/lib)
if(NOT PIGPIO_LIBRARY)
  message(FATAL_ERROR "pigpio library not found")
endif()

# Include directories (include /usr/local/include for pigpio headers)
include_directories(include /usr/local/include)

# Add executable
add_executable(motor_control_node src/motor_control_node.cpp)
target_include_directories(motor_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(motor_control_node ${PIGPIO_LIBRARY})

# Link dependencies
ament_target_dependencies(motor_control_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Install executable
install(TARGETS motor_control_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()