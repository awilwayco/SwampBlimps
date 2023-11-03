#!/bin/bash

# Also can use an alias 'rv' from anywhere on Steve
#Make rv1, rv2, rv3, ... for individual startups

cd ~/Vision/Computation
Cameras/./runCamera1.sh &
sleep 0.1

cd ROS2_Stereo_Launch/Camera1
./ros2StereoLaunch1.sh &
sleep 0.1

cd ../../ML
./runMLNode1.sh

read -r -d '' _ </dev/tty

