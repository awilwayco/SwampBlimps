#!/bin/bash

ros2 launch opencv_telemetry split_node.launch.py namespace:=TurboBlimp calibration_file:=camera3 camera_ns:=TurboBlimp

