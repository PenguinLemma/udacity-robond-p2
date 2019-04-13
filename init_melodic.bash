#!/usr/bin/env bash
# To initialize the catkin_workspace and set some environment variables
# that are necessary.

# Build the catkin ws
catkin_make

# Set the needed environment variables
source devel/setup.bash
# In case of using ROS Melodic, the following command is also necessary,
# due to: https://github.com/ros-visualization/rviz/issues/1249
export LC_NUMERIC="en_US.UTF-8"
# Add path where models in the world can be found
export GAZEBO_MODEL_PATH=$PWD/src/my_robot/worlds/model:~/.gazebo/models:$GAZEBO_MODEL_PATH