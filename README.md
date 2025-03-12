# stage_ros

## Setup

### ClangFormat

**Installation**

```sh
sudo apt update
sudo apt install -y clang-format
```

**Format Configuration File**
```sh
# If you want to create a format configuration file for the first time...
# https://clang.llvm.org/docs/ClangFormat.html
clang-format --style=<llvm|google|chromium|mozilla|webKit> --dump-config > .clang-format
```

**Visual Studio Code**

1. Install the C/C++ extension (ms-vscode.cpptools)
2. View > Command Pallet... (`Ctrl+Shift+P`)
   - Preferences: Open User Settings (JSON)
3. Paste the following in your user settings JSON file
   ```json
   {
     "editor.formatOnSave": true,
     "[cpp]": {
       "editor.defaultFormatter": "ms-vscode.cpptools"
     },
     "C_Cpp.clang_format_style": "file",
     "C_Cpp.formatting": "clangFormat"
   }
   ```

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
sudo apt-get install -y libjpeg-dev libpng-dev libltdl-dev libfltk1.3-dev libglu1-mesa-dev

mkdir ~/dev -p
cd ~/dev
git clone https://github.com/sousarbarb/Stage.git

cd Stage
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/stage ..
make
sudo make install

export LD_LIBRARY_PATH=/opt/stage/lib:$LD_LIBRARY_PATH

cd /opt/stage/bin/
./stage ../share/stage/worlds/simple.world
```

**ROS 1 Installation**
```sh
sudo apt install -y ros-`echo $ROS_DISTRO`-stage
```

## Usage

### Build

**ROS 1**
```sh
source /opt/ros/noetic/setup.bash

mkdir ~/ros1_ws/src -p

cd ~/ros1_ws/src/
git clone git@github.com:sousarbarb/stage_ros.git

cd ~/ros1_ws/
catkin_make --force-cmake --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**ROS 2**
```sh
source /opt/ros/foxy/setup.bash

mkdir ~/ros2_ws/src -p

cd ~/ros2_ws/src/
git clone git@github.com:sousarbarb/stage_ros.git

cd ~/ros2_ws/
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers summary+ status+ console_direct+ console_start_end+ console_stderr+
```
