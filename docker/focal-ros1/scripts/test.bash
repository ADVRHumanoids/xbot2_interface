#!/bin/bash
set -e

# setup env
source /opt/ros/noetic/setup.bash
source ~/test_ws/setup.bash 

# run tests
cd ~/test_ws/build/xbot2_interface
ctest --output-on-failure
