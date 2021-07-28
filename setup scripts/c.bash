#!/bin/bash

# c stands for colcon

source /opt/ros/dashing/setup.bash
source /opt/openrobots/setup.bash
cd ~/Desktop/dev/workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF
source ~/Desktop/dev/workspace/install/setup.bash
