#!/bin/bash

# Also can use an alias 'rv' from anywhere on Steve
#Make rv1, rv2, rv3, ... for individual startups

cd ~/Vision/Computation
Cameras/./runCamera4.sh &
sleep 0.1

cd ROS2_Stereo_Launch/Camera4
./ros2StereoLaunch4.sh &
sleep 0.1

cd ../../ML
./runMLNode4.sh

read -r -d '' _ </dev/tty

