#!/bin/bash

# Also can use an alias 'rv' from anywhere on Steve
#Make rv1, rv2, rv3, ... for individual startups

cd ~/Vision/Computation
Cameras/./runCamera2.sh &
sleep 0.1

cd ROS2_Stereo_Launch/Camera2
./ros2StereoLaunch2.sh &
sleep 0.1

cd ../../ML
./runMLNode2.sh

