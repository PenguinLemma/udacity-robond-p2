#!/usr/bin/env zsh
# To initialize the catkin_workspace and set some environment variables
# that are necessary.

# Build the catkin ws
catkin_make

# Set the needed environment variables
source devel/setup.zsh
# Add path where models in the world can be found
export GAZEBO_MODEL_PATH=$PWD/src/my_robot/worlds/model:~/.gazebo/models:$GAZEBO_MODEL_PATH
