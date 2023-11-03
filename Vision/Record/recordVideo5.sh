#!/bin/bash

FILENAME=output_$(date +%m-%d-%Y_%I:%M:%S%p).avi
ros2 run image_view video_recorder --ros-args --remap image:=FiveGuysBlimp/left/image_raw -p filename:=$FILENAME
mv $FILENAME ../Recordings

