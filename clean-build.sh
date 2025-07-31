#!/bin/bash

echo "Deleting ros2_ws/build, ros2_ws/install, and ros2_ws/log directories..."
# Clean build artifacts
rm -rf ros2_ws/build/ ros2_ws/install/ ros2_ws/log/
echo "Done. Execute now 'connect.sh' to rebuild the workspace."

