# stage_ros

## Setup

### Stage

**Repository**
- https://github.com/sousarbarb/Stage
- https://github.com/rtv/Stage/ _(original)_

![Stage Build Status](https://github.com/sousarbarb/Stage/actions/workflows/ci.yml/badge.svg)

**Ubuntu Installation**
```sh
sudo apt-get update
sudo apt-get dist-upgrade -y
sudo apt-get install -y git build-essential cmake
# ( if you do not want interactive mode, execute the following
#   before installing the other dependencies... )
# DEBIAN_FRONTEND=noninteractive sudo apt-get install -y --no-install-recommends tzdata
sudo apt-get install -y libjpeg-dev libpng-dev libltdl-dev libfltk1.3-dev libglew-dev

mkdir ~/dev -p
cd ~/dev
git clone https://github.com/sousarbarb/Stage.git

cd Stage
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/stage ..
make
make install

export LD_LIBRARY_PATH=/opt/stage/lib:$LD_LIBRARY_PATH

cd /opt/stage/bin/
./stage ../share/stage/worlds/simple.world
```

**ROS 1 Installation**
```sh
sudo apt install -y ros-`echo $ROS_DISTRO`-stage
```
