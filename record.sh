#!/bin/bash

DEFAULT_TOPICS=(
    "/parameter_events"
    "/rosout"
)

CAMERA_TOPICS=$(ros2 topic list | grep -Ev "$(
    IFS='|'
    echo "${DEFAULT_TOPICS[*]}"
)")

if [ -z "$CAMERA_TOPICS" ]; then
    echo "There are no topics to record."
    exit 1
fi

echo "The following topics will be recorded:"
echo "$CAMERA_TOPICS"

ros2 bag record -a -o ros2_bag_$(date +%Y-%m-%d_%H-%M-%S) $CAMERA_TOPICS
