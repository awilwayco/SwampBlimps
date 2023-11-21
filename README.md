
# Swamp Blimps
![Logo](https://github.com/awilwayco/SwampBlimps/assets/56363833/e8eed94d-2a73-4e07-a487-f3db24324206)

## Authors
- Austin Wilwayco | Email: awilwayco@ufl.edu
- Bryan Ortiz | Email: ortizb@ufl.edu

## Overview
Our team works as part of a research organization called "Swamp Blimps" in a laboratory that creates autonomous 
lighter-than-air (LTA) blimps. Our team's goal is to update the existing basestation by making it web-based for multiple users to access it at once. This will allow the pilot to use the basestation to select and control the blimps while other users can view blimp livestreams, machine learning data, etc.
<p align="center">
  
![Blimp](https://github.com/awilwayco/SwampBlimps/assets/56363833/ff6e067d-5df5-41a7-9ac8-dd0bc9154609)
<p align="center">
<em>Figure 1. A blimp performing an autonomous catch</em>
</p>
</p>

## Summary
This repository contains the basestation code. The basestation is primarily used to view all available blimps and send commands to the currently connected blimp through the use of an Xbox controller or keyboard. A secondary feature of the basestation is the ability to view livestreams from each blimp, which is available through a hyperlink using the stream section of the main page on the basestation.

The basestation uses Python with the Flask framework for the backend and HTML and JavaScript for the frontend. 
<p align="center">
  
![Basestation_MainPage](https://github.com/awilwayco/SwampBlimps/assets/56363833/caebd954-32cd-4be2-afc8-6283b00b8699)
<p align="center">
<em>Figure 2. Basestation UI</em>
</p>
</p>

## Communication
The basestation uses a communication network called ROS 2 to publish and subscribe to data. When data is being published (sent) by a device with ROS 2, other devices with ROS 2 can subscribe (access) that data. For our use case, this allows the basestation to know when a blimp is online since a blimp ID is being published by the blimps, which the basestation subscribes to. This communication network allows for bidirectional communication between the basestation and blimps.

In terms of communication between the frontend and backend for the basestation, we use Flask's SocketIO library to have ROS 2 data appear on our UI.
<p align="center">
  
![Communication](https://github.com/awilwayco/SwampBlimps/assets/56363833/e5745c57-0006-4671-ab2f-39ad85e8a0d8)
<p align="center">
<em>Figure 3. A high-level overview of how the basestation and blimps handle communication</em>
</p>
</p>

## Basestation Requirements

- A device with the Ubuntu 20.04 Operating System (https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- Python 3.8.10 installed on the device (https://www.python.org/downloads/release/python-3810/)
- ROS2-Foxy installed on the device (https://docs.ros.org/en/foxy/Installation.html)
- Pip install the following packages: Rclpy, Flask, Flask-SocketIO, Simple-Websocket, OpenCV-Python, Pyserial, and Numpy

## How to Use Basestation

1. Run “./run.sh” within a terminal to startup basestation.
2. Click the URL link provided in the terminal to navigate to the basestation.
3. Connect a controller to the administrator's computer. 
4. Activate a blimp to connect to the basestation by plugging in batteries for Orange Pi and Teensy. 
5. Connect a controller to the blimp by using the Xbox controller's d-pad’s up or down arrows to select which blimp to connect to. 
6. As an administrator, select the corresponding goal color for catching blimps by toggling the "Goal" button by clicking it with a mouse or pressing “Y” on the controller. For attack blimps, they can toggle the “Target” button by clicking it with a mouse or pressing “X” on the controller. 
7. Activate autonomous mode on the blimp by pressing “RT” on the controller. Press "LT" to send all the blimps into autonomous mode. 
8. To maneuver manually, deactivate autonomous mode by pressing “RT” or "LT" again and using the left and right sticks on the controller. 
9. Click the "View Stream" hyperlink for the corresponding blimp to navigate to the livestream for that blimp. 
10. Use the sidebar menu to navigate to the Main, Logs, Barometer, or Documentation pages.

## Flashing Teensy Wirelessly Requirements
- Teensy 4.0 Microcontroller (https://www.pjrc.com/store/teensy40.html)
- PlatformIO for VSCode (https://platformio.org/install/ide?install=vscode)
- Teensy Loader CLI (https://www.pjrc.com/teensy/loader_cli.html)
- libusb-dev Library (https://packages.ubuntu.com/focal/libusb-dev)

## How to Flash Teensy Wirelessly
1.	In the BlimpV8/BlimpV8_Teensy folder containing the Teensy code, run ./flashTeensy.sh # with the ‘#’ being the number of the orange pi you want to flash. Alternatively, you can run ./allFlashTeensys.sh to flash all Teensys that are currently powered on. Please note that this has been made to run for specific hardware with static IP addresses for security reasons.

## Vision Code Requirements
- A device with the Ubuntu 20.04 Operating System (https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- Python 3.8.10 installed on the device (https://www.python.org/downloads/release/python-3810/)
- ROS2-Foxy installed on the device (https://docs.ros.org/en/foxy/Installation.html)
- YoloV5 installed on the device (https://github.com/ultralytics/yolov5)
- Xdotool package installed on device (How to install xdotool ubuntu package on Ubuntu 20.04/Ubuntu 18.04/Ubuntu 19.04/Ubuntu 16.04 (zoomadmin.com))

## How to Startup Vision Code
1.	In the Vision folder, run ./startVision.sh # with the ‘#’ being the number of the orange pi you want to flash. Please note that this code runs machine learning code that is not available on our GitHub that is not our code to disclose. Terminator is another program required to run this bash script. This script has also been made to run for specific hardware with static IP addresses for security reasons. The use of this bash script can be seen in the YouTube video we provided.
