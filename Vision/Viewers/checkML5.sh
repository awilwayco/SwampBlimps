#!/bin/bash

cd ~
cd ros2_stereo
source install/setup.bash
ros2 topic echo /FiveGuysBlimp/bounding_box

