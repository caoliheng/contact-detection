#!/bin/bash


source /opt/ros/dashing/setup.bash
source /opt/openrobots/setup.bash
cd ~/Desktop/dev/workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/Desktop/dev/workspace/install/setup.bash
