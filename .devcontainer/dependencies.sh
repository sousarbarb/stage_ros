#!/bin/bash
apt-get update
apt-get install -y build-essential gdb cmake
apt-get install -y git
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends tzdata
apt-get install -y libjpeg-dev libpng-dev libltdl-dev libfltk1.3-dev libglu1-mesa-dev

# required for Docker usage with Wayland-based Ubuntu distros (if wayland used...)
# apt-get install -y qtwayland5

mkdir /home/dev -p

cd /home/dev
git clone --branch master https://github.com/sousarbarb/Stage.git

cd Stage
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/stage ..
make -j$(nproc)
sudo make install
sudo ldconfig --verbose /opt/stage/lib/

# mkdir /home/ros_ws/src -p

# cd /home/ros_ws/src

# git clone https://github.com/sousarbarb/stage_ros.git
