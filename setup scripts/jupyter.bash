#!/bin/bash


source /opt/ros/dashing/setup.bash
source /opt/openrobots/setup.bash
source install/setup.bash

cd src/contact-detection
~/.local/bin/jupyter-lab --allow-root --no-browser
