#!/bin/bash


source /opt/ros/dashing/setup.bash
source /opt/openrobots/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source /home/lcao/Desktop/dev/workspace/install/setup.bash
