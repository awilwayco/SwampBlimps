#!/bin/bash

ros2 launch opencv_telemetry split_node.launch.py namespace:=SillyAhBlimp calibration_file:=camera2 camera_ns:=SillyAhBlimp

