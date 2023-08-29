# Swamp Blimps

# Pre-Alpha Build Video Link
- https://youtu.be/ogv464oKUCc

# Completed Work
- Shift towards a robust and secure list of active blimps to avoid the phantom blimp bug.
- Evaluation and recommendation of components to team leads.
- Learned and applied process to flash and program Raspberry Pi.
- Learned and applied process to define static IP address to Raspberry Pi.
- (Implemented Lost State)
- (Assessment of potential software and circuit bugs in regards to controller issue).
- Implemented Goal Alignment Detection using OpenCV.
- Created Prototype of New Basestation using Flask and HTML.

# Project Architecture
Attacking Blimp
- Ultrasonic Sensor sends ultrasonic wave data used for height calculations through wires to Open Wide Angle Lens Camera. Ultrasonic Sensor is configured to use the universal asynchronous receiver / transmitter (UART) protocol for communication to Camera. Camera sends image processing and target recognition data through UART protocol to Teensy Microcontroller. Camera receives both power and ground from Electronic Speed Control (ESC) organized on a circuit board through wiring. Inertial Measurement Unit (IMU) collects gyroscope and accelerometer data and sends results to Teensy Microcontroller through the serial peripheral interface (SPI) protocol. ESC divides voltage passed in to 5 DC V for voltage regulation. ESC also sends direct current pulse width modulation (DC PWM) to ESP32 Microcontroller to control motor speed through wiring on circuit board. Teensy Microcontroller receives data from various components through pins connected to circuit board. The data is then used for internal calculations through programming. These include control calculations, filters, and state machines. ESP01 Microcontroller is used for Wi-Fi connectivity to connect to the router and communicate with the base station, the catching blimp, and other attacking blimps.

Catching Blimp
- Stereo Camera uses image recognition and machine learning to learn colors and identify objects, in this case balloons and goals. The Stereo Camera sends information for calculations and state machine decisions to Raspberry Pi 4 Microprocessor and connects to the Raspberry Pi 4 Microprocessor using a micro-USB cable. The universal asynchronous receiver / transmitter (UART) protocol is used for communication between these devices. The Raspberry Pi 4 Microprocessor passes the data to the Teensy Microcontroller, again using UART. Additionally, the Raspberry Pi 4 Microprocessor offers Wi-Fi connectivity, thus facilitating the communication between the catching blimp, attacking blimps, and the base station. The Raspberry Pi 4 Microprocessor also receives power and ground from the Electronic Speed Control (ESC) from circuit board. An Optical Flow Sensor handles control calculations and passes the information to the Teensy Microcontroller by using the Inter-Integrated Circuit (I2C) protocol. The Electronic Speed Control (ESC) divides voltage passed in to 5 DC V before using Analog-to-Digital Conversion (ADC) to translate voltage from analog to digital to the Teensy Microcontroller. The ESC is also used to control motor speed. Inertial Measurement Unit (IMU) collects gyroscope and accelerometer data and sends data to Teensy Microcontroller through the serial peripheral interface (SPI) protocol. Teensy Microcontroller receives information from various components through its pins on a circuit board. The information is used for internal programming tasks, including control calculations, filters, state machines, and communication between the devices. Despite the Teensy Microcontroller not offering Wi-Fi capabilities like the ESP32 Microcontroller, the need for faster processing speed and greater amount of RAM that the Teensy Microcontroller provides lead to it being chosen over the ESP32 Microcontroller. 

Network Architecture
-  The router defines the blimp's static IP and establishes connection between the base station and other connected blimps. The static IP must be known by the other blimps and basestation for them to receive information from the IP. Messages are sent amongst blimps themselves and to the base station by each blimp. The barometer does not receive messages, but rather only sends messages over the router to the base station for height and pressure analysis. Lastly, all of this connects to a private network on a router that is not connected to the internet for security purposes. 

UDP Architecture
- UDP Packet containing required data that is being sent both between the blimps and basestation and among the blimps themselves. Identifier indicates the message comes from an expected device. TargetID is the ID that transmits the message (based off the IP address’ assigned host ID). SourceID operates as a buffer between the TargetID, flags, and message. The flag section indicates how a device should receive the data, the options being as a data packet, a state change, or a barometer reading. Data indicates parameter message codes which verifies whether the device used to grab balloons is active or not and whether the blimp is in autonomous mode or not. 

# Known Bugs
Phantom Blimp Bug
- A blimp that the user cannot connect to appears.
- System that uses maps for verifying, adding, and removing blimps should offer a robust and secure list of active blimps, removing this bug from potentially occurring.

Camera communication with Raspberry
- Requires enabling (RESOLVED on 3/04/2023).

Depth Calibration Issue
- Improper calculations occurring on depth are resulting in inconsistent results during catching of balloons and approaching goals.
- Issue would be in either the PyStereoVision or PyBlimpVision files.
- (RESOLVED on 4/11/2023) New masking and application of SIFT algorithm seems to have provided acceptable results for competition

Catching Blimp's Raspberry Pi is not recognized by the base station. 
- Raspberry Pi appears as "New Blimp" on Display despite already being assigned a static local IP address.
- Problem due to Raspberry Pi. UDP message displays "N" is being sent as the targetID. 
- (RESOLVED on 3/28/2023) Raspberry Pi program updated. UDP now sends IP address as string variable and is properly understood and displayed by the base station.

ESP32 Microcontroller cannot establish a persistent connection with the base station.
- This is likely due to a mishandling of the UDP message from the Attacking Blimp side. (Note: May not be necessary to fix due to decision to change Attacking Blimp over to the Teensy Microcontroller and a WiFi card).
- (Resolved on 4/12/2023) Transitioned over to Teensy and ESP 01 instead.

Raspberry Pi does not accept Wi-Fi outside of laboratory’s secure router
- Attempted SSH, ethernet cable, unzipping tar.gz files, using sudo raspi-config, and setting internet directly in wpa_supplicant.cnf file, to connect to the internet, but does not accept any Wi-Fi outside of the lab's Wi-Fi

# How to view Documentation folder
1. Download zip file
2. Unzip file
3. Enter into the desired file
4. Enter into the html folder
5. Open the index.html file within the html folder. This should open the file in a local web browser
