cmake_minimum_required(VERSION 3.5)
project(martian_mines)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclpy REQUIRED)

# Message generation (ROS 2 uses `rosidl_generate_interfaces`)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FigureMsg.msg"
  "msg/FigureMsgList.msg"
  "msg/BoundingBoxLabeled.msg"
  "msg/BoundingBoxLabeledList.msg"
  DEPENDENCIES std_msgs vision_msgs
)

# Install Python scripts
install(PROGRAMS
  src/figure_finder.py
  src/precision_landing.py
  src/bbox_publisher.py
  src/detection.py
  src/trajectory_generator.py
  src/report_uploader.py
  src/environment_visualization.py
  src/trajectory_tracker.py
  src/detections_visualization.py
  src/mission_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Include directories
include_directories(
  ${ament_INCLUDE_DIRS}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
