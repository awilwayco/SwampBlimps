#!/bin/bash

ros2 launch opencv_telemetry split_node.launch.py namespace:=FiveGuysBlimp calibration_file:=camera5 camera_ns:=FiveGuysBlimp

