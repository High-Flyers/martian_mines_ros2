cmake_minimum_required(VERSION 3.8)
project(martian_mines_ros2)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED)

# Install Python package
install(
  DIRECTORY martian_mines_ros2
  DESTINATION lib/${PROJECT_NAME}  # Change to lib/python3.x/site-packages/${PROJECT_NAME} if using ament_python
  USE_SOURCE_PERMISSIONS
)

ament_package()
