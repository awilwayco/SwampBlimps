#!/bin/bash

cd ~
source /opt/ros/foxy/setup.bash
source ./ros2_stereo/install/setup.bash
source ./Luke_ML_Blimp/ros2_tracks_ws/install/setup.bash

ros2 launch track_ros2 track_ros2_node.launch.py namespace:=SillyAhBlimp

read -r -d '' _ </dev/tty
