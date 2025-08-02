#!/bin/bash

CONTAINER_ID=$(docker ps -q --filter "name=nautilus-ros2*" | head -n 1)

if [ -n "$CONTAINER_ID" ]; then
    echo "exec"
    docker exec -it "$CONTAINER_ID" /bin/bash
else
    echo "compose"	
    docker compose run --rm raspberry
fi

