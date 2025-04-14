cmake_minimum_required(VERSION 3.5)
project(localization_pkg_py)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Install the Python script(s)
install(PROGRAMS
  src/convert2pointcloud2.py
  src/icp_module.py
  src/icp_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()