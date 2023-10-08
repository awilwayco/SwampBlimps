# Swamp Blimps

## Table of Contents

## Overview
Our team works as part of a research organization in a laboratory that creates autonomous 
lighter-than-air (LTA) blimps. The goal of this project is to update the existing basestation by making it web-based for multiple users to access it at once. This will allow the pilot to use the basestation to select and control the blimps while other users can view blimp livestreams, machine learning data, etc.
<p align="center">
  
![Blimp](https://github.com/awilwayco/SwampBlimps/assets/56363833/ff6e067d-5df5-41a7-9ac8-dd0bc9154609)
<p align="center">
<em>Figure 1. A blimp performing an autonomous catch</em>
</p>
</p>

## Summary
This repository contains the basestation code. The basestation is primarily used to view all available blimps and send commands to the currently connected blimp through the use of an Xbox controller or keyboard. A secondary feature of the basestation is the ability to view livestreams from each blimp, which is available through a hyperlink using the stream section of the main page on the basestation.
<p align="center">
  
![Basestation_Pic](https://github.com/awilwayco/SwampBlimps/assets/56363833/863f8d6f-4043-4ccf-b854-9cc79f418d3e)

<p align="center">
<em>Figure 2. Basestation UI</em>
</p>
</p>

## Communication
The basestation uses a communication network called ROS 2 to publish and subscribe to data. When data is being published (sent) by a device with ROS 2, other devices with ROS 2 can subscribe (access) that data. For our use case, this allows the basestation to know when a blimp is online since a blimp ID is being published by the blimps, which the basestation subscribes to. This communication network allows for bidirectional communication between the basestation and blimps.

In terms of communication between the frontend and backend for the basestation, we use Flask's SocketIO library in order to have ROS 2 data appear on our UI.
<p align="center">
  
![Communication](https://github.com/awilwayco/SwampBlimps/assets/56363833/e5745c57-0006-4671-ab2f-39ad85e8a0d8)
<p align="center">
<em>Figure 3. A high-level overview of how the basestation and blimps handle communication through the use of the ROS 2 Network</em>
</p>
</p>

## How to Install

## How to Use
