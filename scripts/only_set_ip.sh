#!/bin/bash

if [ $# -ge 1 ]; then
    if [ "$1" = "master" ]; then
        MASTER_IP_ADDRESS=$2
        export ROS_DOMAIN_ID=0  # Ensure a common domain ID
        export FASTRTPS_DEFAULT_PROFILES_FILE=""  # Clear previous FastDDS settings
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        echo "ROS 2 Master Node: Using default DDS settings."
        
    elif [ "$1" = "client" ]; then
        MASTER_IP_ADDRESS=$2
        CLIENT_IP_ADDRESS=$3
        export ROS_DOMAIN_ID=0  # Ensure it matches the master node
        export FASTRTPS_DEFAULT_PROFILES_FILE=""
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        echo "ROS 2 Client Node: Using default DDS settings."
    fi
fi

echo "ROS_DOMAIN_ID set to: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION set to: $RMW_IMPLEMENTATION"
