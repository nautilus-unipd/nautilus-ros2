#!/bin/bash

if [ -f /.dockerenv ]; then
    ros2 bag record /front/camera_left/image /front/camera_right/image
else
    echo "Not running in Docker environment, execute 'connect.sh' to run the container."
    exit 1
fi