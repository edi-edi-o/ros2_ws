cmake_minimum_required(VERSION 3.5)
project(imu_phone_data)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(robot_state_publisher REQUIRED) # Added for robot_state_publisher

# Node
add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  std_msgs
  Boost
  robot_state_publisher # Added dependency
)

# If you need to link against Boost libraries explicitly:
target_link_libraries(imu_node Boost::system)

# Install
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()