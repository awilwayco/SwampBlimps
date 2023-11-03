#!/bin/bash

ros2 launch opencv_telemetry split_node.launch.py namespace:=GameChamberBlimp calibration_file:=camera4 camera_ns:=GameChamberBlimp

