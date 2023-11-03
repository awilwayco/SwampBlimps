#!/bin/bash

# Also can use an alias 'rv' from anywhere on Steve
#Make rv1, rv2, rv3, ... for individual startups

cd ~/Vision/Computation
Cameras/./runCamera3.sh &
sleep 0.1

cd ROS2_Stereo_Launch/Camera3
./ros2StereoLaunch3.sh &
sleep 0.1

cd ../../ML
./runMLNode3.sh

read -r -d '' _ </dev/tty

