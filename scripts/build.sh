#!/bin/sh
set -e

if [ -n "$1" ]; then
    cd "$1"
fi

SYMLINK_FLAG=""

if [ "$2" = "--symlink-install" ]; then
    SYMLINK_FLAG="--symlink-install"
fi

colcon build ${SYMLINK_FLAG} \
    --packages-skip \
        martian_mines_msgs \
        px4_msgs \
        microxrcedds_agent \
        realsense2_camera_msgs \
        realsense2_camera \
        realsense2_description

colcon build martian_mines_msgs
