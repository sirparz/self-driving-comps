cmake_minimum_required(VERSION 3.5)
project(localization_pkg_py)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install launch files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# ✅ Install Python scripts from the 'src' folder
install(
  PROGRAMS
    src/convert2pointcloud2.py
    src/icp_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
