#!/bin/bash


source /opt/ros/dashing/setup.bash
source /opt/openrobots/setup.bash
source ~/Desktop/dev/workspace/install/setup.bash

cd ~/Desktop/dev/workspace/src/contact_detection
~/.local/bin/jupyter-lab --allow-root --no-browser
