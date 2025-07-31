#!/bin/bash

# change this when changes form the autodocking branch are merged
# CONTAINER_ID=$(docker ps -q --filter "ancestor=ghcr.io/nautilus-unipd/raspberry-setup:latest" | head -n 1)
CONTAINER_ID=$(docker ps -q --filter "rasp-test" | head -n 1)

if [ -n "$CONTAINER_ID" ]; then
    echo "exec"
    docker exec -it "$CONTAINER_ID" /bin/bash -c "cd /home/ubuntu/nautilus-ros2"
else
    echo "compose"	
    docker compose run --rm raspberry
fi

