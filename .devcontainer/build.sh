#!/bin/bash
cd /home/ros_ws/src/stage_ros

rm -rf build/
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/stage ..
make -j$(nproc)
sudo make install
sudo ldconfig --verbose /opt/stage/lib/
