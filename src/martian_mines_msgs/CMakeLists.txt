cmake_minimum_required(VERSION 3.22)
project(martian_mines_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/BoundingBoxLabeled.msg
  msg/BoundingBoxLabeledList.msg
  msg/FigureMsg.msg
  msg/FigureMsgList.msg
  msg/ENULocalOdometry.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES vision_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()