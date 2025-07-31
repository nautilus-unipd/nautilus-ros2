#!/bin/bash

if [ -f /.dockerenv ]; then
    cd /home/ubuntu/nautilus-ros2/ros2_ws
    source ./install/setup.bash
    ros2 launch camera_module stereo_camera_front.launch.py
else
    echo "Not running in Docker environment, execute 'connect.sh' to run the container."
    exit 1
fi