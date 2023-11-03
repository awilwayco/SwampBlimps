#!/bin/bash

cd ~
source /opt/ros/foxy/setup.bash
source ~/ros2_stereo/install/setup.bash
source ~/Luke_ML_Blimp/ros2_tracks_ws/install/setup.bash

./ros2_stereo/gstreamerNode_TurboBlimp.sh

read -r -d '' _ </dev/tty
