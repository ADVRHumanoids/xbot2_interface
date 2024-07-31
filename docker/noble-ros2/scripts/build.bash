#!/bin/bash
set -e

# chown to user (needed after mounting volumes)
sudo chown -R user:user ~/test_ws

# pip deps
sudo pip install hhcm-forest --break-system-packages
sudo pip install setuptools --break-system-packages

# make forest https
export HHCM_FOREST_CLONE_DEFAULT_PROTO=https

# create ws and source it
mkdir -p ~/test_ws && cd ~/test_ws
forest init

# setup env
source /opt/ros/jazzy/setup.bash
source setup.bash

# add recipes
forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git -t ros2

# build
export PYTHONUNBUFFERED=1
forest grow xbot2_interface --verbose --clone-depth 1

# build tests
cd build/xbot2_interface
cmake -DXBOT2_IFC_BUILD_TESTS=1 .
make
