#!/bin/bash

# Unset ROS 1 specific environment variables
unset ROS_MASTER_URI
unset ROS_IP

# Build the ROS 2 workspace using colcon
colcon build
source install/setup.bash

# Set up ROS 2 network configuration
if [ $# -ge 1 ]; then
    if [ "$1" = "master" ]; then
        MASTER_IP_ADDRESS=$2
        export ROS_DOMAIN_ID=0  # Ensure all nodes in the network share the same domain ID
        export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros2/master.xml  # Optional: Define specific QoS settings

    elif [ "$1" = "client" ]; then
        MASTER_IP_ADDRESS=$2
        CLIENT_IP_ADDRESS=$3
        export ROS_DOMAIN_ID=0
        export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/ros2/client.xml  # Optional: Define specific QoS settings
    fi
fi

echo "ROS 2 Domain ID set to: $ROS_DOMAIN_ID"
