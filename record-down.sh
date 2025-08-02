#!/bin/bash

if [ -f /.dockerenv ]; then
    ros2 bag record /down/camera_left/image /down/camera_right/image
else
    echo "Not running in Docker environment, execute 'connect.sh' to run the container."
    exit 1
fi