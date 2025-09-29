#!/bin/bash
cp .devcontainer/ros1noetic/c_cpp_properties.json .vscode/c_cpp_properties.json

source /opt/ros/noetic/setup.bash

cd /home/ros_ws/
catkin_make --force-cmake --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
