#!/bin/bash

CONTAINER_ID=$(docker ps -q --filter "name=nautilus-ros2*" | head -n 1)

# connect to an alredy running instance
if [ -n "$CONTAINER_ID" ]; then
    echo "Connecting to running instance.\n"
    docker exec -it "$CONTAINER_ID" /bin/bash
else # build or start the container
    echo "Starting new instance.\n"
    docker compose run --rm raspberry
fi

