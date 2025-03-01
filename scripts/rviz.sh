#!/bin/bash

cd $ROS_WORKSPACE
catkin build
source devel/setup.bash
ros2 launch martian_mines rviz.launch