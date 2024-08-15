cmake_minimum_required(VERSION 3.8)
project(indv_prj_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)


find_package(interfaces_indv_prj_pkg REQUIRED)                   

add_executable(drone_thruster_pub src/drone_thruster_pub.cpp)
ament_target_dependencies(drone_thruster_pub rclcpp interfaces_indv_prj_pkg) 

# add_executable(clock_publisher src/clock_publisher.cpp)
# ament_target_dependencies(clock_publisher rclcpp std_msgs) 


add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(path_publisher rclcpp std_msgs nav_msgs geometry_msgs)  

add_executable(curve_path_publisher src/curve_path_publisher.cpp)
ament_target_dependencies(curve_path_publisher rclcpp std_msgs nav_msgs geometry_msgs)  


add_executable(depthConverter src/depthConverter.cpp)
ament_target_dependencies(depthConverter rclcpp std_msgs sensor_msgs cv_bridge)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp nav_msgs geometry_msgs tf2_ros) 

include_directories(include)

install(TARGETS
drone_thruster_pub
# clock_publisher
path_publisher
curve_path_publisher
depthConverter
tf_broadcaster
DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  # scripts/RGBDreceiver.py
  # scripts/clock_pub.py
  # scripts/csvplot.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
