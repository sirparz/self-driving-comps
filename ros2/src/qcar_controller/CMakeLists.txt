cmake_minimum_required(VERSION 3.8)
project(qcar_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python scripts as executables
install(
  PROGRAMS
    src/wasd_teleop.py  # Ensure this matches the actual location of your script
    src/logitech_g923_controller.py  # Ensure this matches the actual location of your script
    src/test_logitech_g923.py  # Ensure this matches the actual location of your script
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint checks (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()