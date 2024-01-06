#!/bin/bash

source ../install/setup.sh

pose=$2

if [ -z "$pose" ]; then
    echo "Usage: ./nav_to_pose.sh \"[x, y, theta]\""
    exit 1
fi

ros2 run pothole_finder nav_to_pose --ros-args -p "pose:=$pose"