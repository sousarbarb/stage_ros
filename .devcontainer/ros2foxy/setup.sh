#!/bin/bash
cp .devcontainer/ros2foxy/c_cpp_properties.json .vscode/c_cpp_properties.json

source /opt/ros/foxy/setup.bash

cd /home/ros_ws/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers status+ console_direct+ console_start_end+
source install/setup.bash
