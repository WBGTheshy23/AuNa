cmake_minimum_required(VERSION 3.5)
project(auna_rongshi)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(auna_common REQUIRED)
find_package(auna_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(etsi_its_msgs REQUIRED)
find_package(etsi_its_msgs_utils REQUIRED)
####################################################
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
#####################################################

# Include Cpp "include" directory
include_directories(include)
include_directories(include ${EIGEN3_INCLUDE_DIR})

# Add libraries

# Create Cpp executable

# Target dependencies

#################################################
add_executable(robot_marker_publisher src/robot_marker_publisher/robot_marker_publisher.cpp)
add_executable(robot_polygon_publisher src/robot_marker_clustering/robot_polygon_publisher.cpp)
add_executable(robot_marker_clustering
  src/robot_marker_clustering/robot_marker_clustering.cpp
  src/robot_marker_clustering/dbscan.cpp
)
add_executable(robot_mot src/robot_mot/robot_mot.cpp)
add_executable(robot_pose_error src/robot_pose_error/robot_pose_error.cpp)

ament_target_dependencies(robot_pose_error 
  rclcpp 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)

ament_target_dependencies(robot_mot
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(robot_polygon_publisher
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(robot_marker_clustering 
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(robot_marker_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  auna_msgs
)
##################################################

# Link libraries

###################################################
install(TARGETS
  robot_marker_publisher
  robot_marker_clustering
  robot_polygon_publisher
  robot_mot
  robot_pose_error
  DESTINATION lib/${PROJECT_NAME}
)
###############################################


install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Cpp executables
install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()