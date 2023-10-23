#!/usr/bin/env python3

# Flask Packages
from flask import Flask, render_template, request, Response
from flask_socketio import SocketIO

# ROS Packages
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int64, Bool, Float64, Float64MultiArray
from yolo_msgs.msg import BoundingBox # type: ignore

# Livestream Packages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Other Packages
import threading
import time
import sys
import signal
import os
import serial
import numpy as np
import json
import subprocess

# Blimp Class
from blimp import Blimp

# Initialize Flask App and SocketIO
app = Flask(__name__)
socketio = SocketIO(app)

# Get the PID of the current process
current_pid = os.getpid()

# Initialize Barometer (Do a try/catch?)
# barometer = serial.Serial('/dev/ttyACM0', 115200) changes need to fix
# barometer = serial.Serial('/dev/ttyACM1', 115200)

class Basestation(Node):
    def __init__(self):
        super().__init__('Basestation')

        # Number of Blimps
        self.num_blimps = 0

        # Connected Blimp Nodes
        self.connected_blimps = []

        # Blimp Node Handlers
        self.blimp_node_handlers = []

        # Create timer
        self.loopSpeed = 10
        timer_period = 1.0/self.loopSpeed
        self.timer = self.create_timer(timer_period, self.timerLoop)
        self.timeout = 3
        
        # Identify Topic for Teensy to Check if Basestation is On
        self.topicName_identify = "/identify"
        self.identify_sub = self.create_subscription(String, self.topicName_identify, self.identify_callback, 10)
    
    def identify_callback(self, msg):
        global blimps

        # Identify message is just a string with the blimp ID
        blimp_id = msg.data
        if blimp_id in self.connected_blimps:
            # Update last online here because update is only called while the node is subscribed
            blimps[blimp_id].last_online = self.get_clock().now()
        else:
            # Otherwise, check its validity and create a handler for it
            # Testing
            # self.get_logger().info('Node Name: "%s"' % nodeName)
            # Make sure node name is valid
            if self.check_node_name(blimp_id) == True:
                print("ADDING BLIMP")
                self.create_blimp_node_handler(blimp_id)
                blimps[blimp_id].last_online = self.get_clock().now()

    def update_blimp_node_handlers(self):
        global blimps

        # Finally, check for subscription timeout for all connected blimps
        for blimp_node_handler in self.blimp_node_handlers:
            if self.getElapsedTime(blimps[blimp_node_handler.blimp_id].last_online) > self.timeout:
                print("REMOVING BLIMP")
                self.remove_blimp_node_handler(blimp_node_handler.blimp_id)
            else:
                # Run handler update routine
                blimp_node_handler.update()
    
    def create_blimp_node_handler(self, blimp_id):
        
        # Create New Blimp Node Handler
        new_blimp_node_handler = BlimpNodeHandler(self, blimp_id)

        # State Machine Topic
        topic_state_machine = "/" + blimp_id + "/state_machine"

        # Subscribe to the State Machine Topic
        new_blimp_node_handler.sub_state_machine = self.create_subscription(Int64, topic_state_machine, new_blimp_node_handler.state_machine_callback, 10)

        # Image Raw Topic
        topic_image_raw = "/" + blimp_id + "/left/image_raw"

        # Subscribe to the Image Raw Topic
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        new_blimp_node_handler.sub_image_raw = self.create_subscription(Image, topic_image_raw, new_blimp_node_handler.image_raw_callback, qos_profile)

        # Check for Bounding Box Topic
        topic_bounding_box = "/" + blimp_id + "/bounding_box"

        # Subscribe to the Bounding Box Topic
        new_blimp_node_handler.sub_bounding_box = self.create_subscription(BoundingBox, topic_bounding_box, new_blimp_node_handler.bounding_box_callback, 10)

        # Check for Base Barometer Topic
        # topic_baseBarometer = "/" + blimp_id + "/baseBarometer"

        # Subscribe to the State Machine Topic
        # new_blimp_node_handler.sub_baseBarometer = self.create_subscription(Float64, topic_baseBarometer, new_blimp_node_handler.baseBarometer_callback, qos_profile)

        # self.connectBlimp(new_blimp_node_handler)
        new_blimp_node_handler.connect_blimp()

        # Add New Blimp Node to Connected Blimp Nodes
        self.connected_blimps.append(new_blimp_node_handler.blimp_id)

        # Add Blimp Node Handler to List
        self.blimp_node_handlers.append(new_blimp_node_handler)

    def remove_blimp_node_handler(self, blimp_id):
        global blimps

        # Timeout Detected for Blimp Node
        if blimp_id is not None:
            self.get_logger().info('Detected timeout of blimp ID "%s"' % blimp_id)

            # First, get the handle of the node handler
            handler_found = False
            handler_id = None
            for id, handler in enumerate(self.blimp_node_handlers):
                if handler.blimp_id == blimp_id:
                    handler_found = True
                    handler_id = id

            if not handler_found:
                self.get_logger().error('Blimp ID "%s" Handler not found!' % blimp_id)
                return
            
            # Now, gracefully shut down the node handler
            # Destroy subscribers (publishers will timeout anyways)
            self.blimp_node_handlers[handler_id].destroy_subscribers()
            self.blimp_node_handlers[handler_id].destroy_publishers()

            # Remove blimp from list of connected blimps
            self.connected_blimps.remove(blimp_id)

            # Finally, delete the handler object for good
            del self.blimp_node_handlers[handler_id]

            # Delete from global blimps dict
            del blimps[blimp_id]

            self.num_blimps -= 1

            # Remove blimp from frontend display
            socketio.emit('remove', blimp_id)

    # Check if Node Name is valid
    def check_node_name(self, node_name):
        # Todo!!!: Search for node name in global dictionary
        if node_name == 'BurnCreamBlimp' or node_name == 'SillyAhBlimp' or node_name == 'TurboBlimp' or node_name == 'GameChamberBlimp' or node_name == 'FiveGuysBlimp' or node_name == 'Catch2' or node_name == 'Catch1' or node_name == 'Attack1' or node_name == 'Attack2':
            return True
        else:
            return False
        
    # Barometer
    # def updateBarometer(self):
    #     global blimps
    #     try:
    #         data = barometer.readline()  # Read a line of data from the serial port
    #         print(data.decode('utf-8'))  # Assuming data is encoded as UTF-8
    #         for blimp in blimps:
    #             blimps[blimp].barometer = float(data.decode('utf-8'))
    #     except KeyboardInterrupt:
    #         barometer.close()  # Close the serial port on Ctrl+C

    # Timer Functions #
    
    def timerLoop(self):
        # Update Blimp Nodes
        self.update_blimp_node_handlers()

        # Currently not being used
        # Publish Node Topics
        # for blimp_node_handler in self.blimp_node_handlers:
        #     blimp_node_handler.publish()

    def getElapsedTime(self, prevTime):
        elapsedTime = self.get_clock().now() - prevTime
        elapsedTimeSec = elapsedTime.nanoseconds / 10**9
        return elapsedTimeSec

class BlimpNodeHandler:
    def __init__(self, parent_node, node_name):
        self.parent_node = parent_node
        self.blimp_id = node_name
        self.blimp_name = self.get_blimp_name(self.blimp_id)
        self.time_created = self.parent_node.get_clock().now()

        self.identified_count = 0

        # Create publishers now
        self.create_publishers()

        # Initialize all subscribers to None; parent will handle their creation
        self.sub_state_machine = None
        self.sub_image_raw = None
        self.sub_bounding_box = None
        self.sub_baseBarometer = None

        # Livestream Most Recent Frame
        self.frame = None
        self.bridge = CvBridge()

    def connect_blimp(self):
        global blimps
        self.parent_node.get_logger().info('Identified Blimp with ID "%s"' % str(self.blimp_id))
        self.parent_node.num_blimps += 1

        # Create a Blimp object for storing the blimp state
        # Todo!!!: fix redundancies between Blimp objects and handlers (they're one-to-one)
        blimp = Blimp(self.blimp_id)
        blimp.blimp_name = self.blimp_name

        # Set the blimp type
        self.get_blimp_type(blimp)
        blimps[self.blimp_id] = blimp

    def update(self):
        # Todo!!!: Change frequency of each publisher!
        # self.parent_node.get_logger().info("Updating")
        global blimps
        if self.blimp_id in blimps:
            # Emit the blimp data to the webpage
            socketio.emit('update', blimps[self.blimp_id].to_dict())
        
            # Publish the target and/or goal color values to ROS
            self.publish_motorCommands()

            # Need to publish this as needed
            self.publish_auto()

            # Publish barometer data
            # self.publish_barometer()

            if (self.get_blimp_type(blimps[self.blimp_id]) == True):
                # If blimp is an attack blimp, publish the target color
                self.publish_target_color()
            else:
                # Otherwise, publish catching blimp stuff
                self.publish_goal_color()

                # Need to publish these as needed!
                self.publish_grabbing()
                self.publish_shooting()

    def create_publishers(self):
        topic_auto =            "/" + self.blimp_id + "/auto"
        topic_goal_color =      "/" + self.blimp_id + "/goal_color"
        topic_target_color =      "/" + self.blimp_id + "/target_color"
        topic_killed =          "/" + self.blimp_id + "/killed"
        topic_motorCommands =  "/" + self.blimp_id + "/motorCommands"
        topic_grabbing =        "/" + self.blimp_id + "/grabbing"
        topic_shooting =        "/" + self.blimp_id + "/shooting"
        # topic_baseBarometer =   "/" + self.nodeName + "/baseBarometer"

        bufferSize = 1
        self.pub_auto = self.parent_node.create_publisher(Bool, topic_auto, bufferSize)
        self.pub_goal_color = self.parent_node.create_publisher(Int64, topic_goal_color, bufferSize)
        self.pub_target_color = self.parent_node.create_publisher(Int64, topic_target_color, bufferSize)
        self.pub_killed = self.parent_node.create_publisher(Bool, topic_killed, bufferSize)
        self.pub_motorCommands = self.parent_node.create_publisher(Float64MultiArray, topic_motorCommands, bufferSize)
        self.pub_grabbing = self.parent_node.create_publisher(Bool, topic_grabbing, bufferSize)
        self.pub_shooting = self.parent_node.create_publisher(Bool, topic_shooting, bufferSize)
        # self.pub_baseBarometer = self.parent_node.create_publisher(Float64, topic_baseBarometer, bufferSize)

    def destroy_publishers(self):
        self.pub_auto.destroy()
        self.pub_goal_color.destroy()
        self.pub_target_color.destroy()
        self.pub_killed.destroy()
        self.pub_motorCommands.destroy()
        self.pub_grabbing.destroy()
        self.pub_shooting.destroy()
        # self.pub_baseBarometer.destroy()

    # Function to destroy subscribers for elegant behavior
    def destroy_subscribers(self):
        self.parent_node.destroy_subscription(self.sub_state_machine)
        self.parent_node.destroy_subscription(self.sub_image_raw)
        self.parent_node.destroy_subscription(self.sub_bounding_box)
        self.parent_node.destroy_subscription(self.sub_baseBarometer)

    # Continually Poll State Machine Data from Teensy
    def state_machine_callback(self, msg):
        global blimps
        if self.blimp_id in blimps:
            blimps[self.blimp_id].state_machine = msg.data

    # Continually Poll Image Raw Data from Pi
    def image_raw_callback(self, msg):
        global blimps
        if self.blimp_id in blimps:
            try:
                # Convert the ROS Image message to a CV2 image (numpy ndarray)
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.frame = cv_image

                if self.frame is not None:
                    flag, jpeg = cv2.imencode('.jpg', self.frame)
                    blimps[self.blimp_id].frame = jpeg
                
                # Uncomment the following line if you want to see the image using OpenCV
                # cv2.imshow("Received Image", cv_image)
                # cv2.waitKey(1)

            except Exception as e:
                self.parent_node.get_logger().error(f"Failed to convert image: {e}")

    # Continually Poll State Machine Data from Teensy
    def bounding_box_callback(self, msg):
        global blimps
        bb_dict = self.bounding_box_to_dict(msg)
        if self.blimp_id in blimps:
            if bb_dict is not None:
                blimps[self.blimp_id].bounding_box = bb_dict
                # Emit the bounding box data to the webpage
                socketio.emit('bounding_box', blimps[self.blimp_id].bounding_box)

    # Continually Poll State Machine Data from Teensy
    # def baseBarometer_callback(self, msg):
    #     global blimps
    #     try:
    #         data = barometer.readline()  # Read a line of data from the serial port
    #         # print(data.decode('utf-8'))  # Assuming data is encoded as UTF-8
    #         for blimp in blimps:
    #             blimps[blimp].barometer = float(data.decode('utf-8'))
    #     except:
    #         pass

    def bounding_box_to_dict(self, bb_msg):
        """
        Convert BoundingBox message to Python dictionary.

        :param bb_msg: BoundingBox message instance.
        :type bb_msg: yolo_msgs.msg.BoundingBox
        :return: Dictionary representation of the message.
        :rtype: dict
        """
        return {
            "header": {
                "stamp": {
                    "sec": bb_msg.header.stamp.sec,
                    "nanosec": bb_msg.header.stamp.nanosec
                },
                "frame_id": bb_msg.header.frame_id
            },
            "balloon": {
                "x_center": bb_msg.x_center_balloon,
                "y_center": bb_msg.y_center_balloon,
                "width": bb_msg.width_balloon,
                "height": bb_msg.height_balloon
            },
            "y_goal": {
                "x_center": bb_msg.x_center_y_goal,
                "y_center": bb_msg.y_center_y_goal,
                "width": bb_msg.width_y_goal,
                "height": bb_msg.height_y_goal
            },
            "o_goal": {
                "x_center": bb_msg.x_center_o_goal,
                "y_center": bb_msg.y_center_o_goal,
                "width": bb_msg.width_o_goal,
                "height": bb_msg.height_o_goal
            }
        }

    # Used for Attack Blimps Only
    def get_blimp_type(self, blimp):
        # Only need to check for Attack Blimps since default is catching type (0)
        if self.blimp_id == 'Attack1':
            blimp.blimp_type = 1
        elif self.blimp_id == 'Attack2':
            blimp.blimp_type = 1

    def get_blimp_name(self, blimp_id):
        # Blimp Name not Recognized (This should not happen!)
        # self.blimp_id = 'Error'
        blimp_names_by_id = {
            # Catching Blimps #
            'BurnCreamBlimp': 'Burn Cream Blimp',
            'SillyAhBlimp': 'Silly Ah Blimp',
            'TurboBlimp': 'Turbo Blimp',
            'GameChamberBlimp': 'Game Chamber Blimp',
            'FiveGuysBlimp': 'Five Guys Blimp',
            # Fake Blimps #
            'Catch1': 'Catch 1',
            'Catch2': 'Catch 2',
            # Attacking Blimps #
            'Attack1': 'Attack 1',
            'Attack2': 'Attack 2'
        }

        return blimp_names_by_id[blimp_id]

    def publish_target_color(self):
        global blimps
        # Publish goal color value to the ROS topic
        msg = Int64()
        msg.data = blimps[self.blimp_id].target_color
        self.pub_target_color.publish(msg)

    def publish_goal_color(self):
        global blimps
        # Publish goal color value to the ROS topic
        msg = Int64()
        msg.data = blimps[self.blimp_id].goal_color
        self.pub_goal_color.publish(msg)

    def publish_motorCommands(self):
        global blimps
        # Publish motor commands value to the ROS topic
        msg = Float64MultiArray()
        msg.data = blimps[self.blimp_id].motorCommands
        self.pub_motorCommands.publish(msg)

    def publish_auto(self):
        global blimps
        # Publish auto value to the ROS topic
        msg = Bool()
        msg.data = blimps[self.blimp_id].auto
        self.pub_auto.publish(msg)

    def publish_grabbing(self):
        global blimps
        # Publish grabbing value to the ROS topic
        msg = Bool()
        msg.data = blimps[self.blimp_id].grabbing
        self.pub_grabbing.publish(msg)

    def publish_shooting(self):
        global blimps
        # Publish shooting value to the ROS topic
        msg = Bool()
        msg.data = blimps[self.blimp_id].shooting
        self.pub_shooting.publish(msg)

    # def publish_barometer(self):
    #     global blimps
    #     # Publish barometer value to the ROS topic
    #     msg = Float64()
    #     msg.data = blimps[self.blimp_name].barometer
    #     self.pub_baseBarometer.publish(msg)

    # Update Total Disconnection
    @socketio.on('kill_basestation')
    def kill_basestation():
        try:
            print('\nDestroying Basestation Node...\n')
            global node
            node.destroy_node()
            rclpy.shutdown()
            # barometer.close()
            sys.exit(0)
        except SystemExit:
            print('\nTerminating Program...\n')
            os.kill(current_pid, signal.SIGTERM)

    # Update Blimp Class with Dictionary Data
    @socketio.on('update_blimp_dict')
    def update_blimp_dict(data):
        global blimps
        blimp_id = data['blimp_id']
        blimps[blimp_id] = data[blimp_id]

    # Update Connection
    @socketio.on('update_connection')
    def update_connection(data):
        global blimps
        blimps[data].connected = True

    # Update Disconnection
    @socketio.on('update_disconnection')
    def update_disconnection(data):
        global blimps
        blimps[data].connected = False

    # Update Total Disconnection
    @socketio.on('update_total_disconnection')
    def update_total_disconnection():
        global blimps
        for blimp in blimps:
            blimps[blimp].connected = False

    # Update Motor Commands
    @socketio.on('update_motorCommands')
    def update_motorCommands(data):
        #print('\n')
        array = np.frombuffer(data, dtype=np.float64)
        motorCommands = array.tolist()
        #print('Received Data:', motorCommands)
        #print('\n')
        # Iterate through which blimp_name is connected
        global blimps
        for blimp in blimps:
            if blimps[blimp].connected == True:
                blimps[blimp].motorCommands = motorCommands
                #print(blimps[blimp].motorCommands)
            else:
                blimps[blimp].motorCommands = [0.0, -0.0, 0.0, -0.0]
                #print(blimps[blimp].motorCommands)

    # Update All Goal Colors
    @socketio.on('update_all_goal_colors')
    def update_goal_colors():
        global blimps
        global all_goal_color

        all_goal_color = not all_goal_color

        for blimp in blimps:
            blimps[blimp].goal_color = 1 if all_goal_color else 0

    # Update Grabbing
    @socketio.on('update_grabbing')
    def update_grabbing(data):
        global blimps
        blimps[data].grabbing = not blimps[data].grabbing

    # Update Shooting
    @socketio.on('update_shooting')
    def update_shooting(data):
        global blimps
        blimps[data].shooting = not blimps[data].shooting

    # Update Shooting
    @socketio.on('update_auto')
    def update_auto(data):
        global blimps
        blimps[data].auto = not blimps[data].auto

    # Update Autonomous Mode
    @socketio.on('update_auto_panic')
    def update_auto_panic():
        global blimps
        global auto_panic
        for blimp in blimps:
            if auto_panic == False:
                blimps[blimp].auto = True
            elif auto_panic == True:
                blimps[blimp].auto = False
        auto_panic = not auto_panic

    # Update Target Color
    @socketio.on('update_target_color')
    def update_target_color(data):
        global blimps
        blimp_id = data['blimp_id']
        blimps[blimp_id].target_color = data['target_color']

    # Update Goal Color
    @socketio.on('update_goal_color')
    def update_goal_color(data):
        global blimps
        blimp_id = data['blimp_id']
        blimps[blimp_id].goal_color = data['goal_color']

# Handle user connection to webpage
@socketio.on('connect')
def handle_connect():
    print('Client connected with IP:', request.remote_addr)

# Main Basestation Page
@app.route('/')
def index():
    client_ip = request.remote_addr
    return render_template('main.html', client_ip=client_ip)

# Streaming Feeds/Pages
def generate(feed_name):
    while True:
        # Yield the output frame in the byte format
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(blimps[feed_name].frame) + b'\r\n')

@app.route('/video_feed/<string:feed_name>')
def video_feed(feed_name):
    global blimps
    if feed_name in blimps:
        if blimps[feed_name].frame is not None:
            return Response(generate(feed_name), mimetype='multipart/x-mixed-replace; boundary=frame')
        else:
            return Response(status=204)
    else:
        return Response(status=204)

@app.route('/BurnCreamBlimp')
def burnCreamBlimpPage():
    return render_template('BurnCreamBlimp.html')

@app.route('/SillyAhBlimp')
def sillyAhhBlimpPage():
    return render_template('SillyAhBlimp.html')

@app.route('/TurboBlimp')
def turboBlimpPage():
    return render_template('TurboBlimp.html')  

@app.route('/GameChamberBlimp')
def gameChamberBlimpPage():
    return render_template('GameChamberBlimp.html')

@app.route('/FiveGuysBlimp')
def fiveGuysBlimpPage():
    return render_template('FiveGuysBlimp.html')

@app.route('/Catch1')
def catch1Page():
    return render_template('Catch1.html')

@app.route('/Catch2')
def catch2Page():
    return render_template('Catch2.html')

# ROS 2 Thread
def ros_thread():
    rclpy.init()

    global node
    node = Basestation()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

# Terminate Code
def terminate(signal, frame):
    print('\nDestroying Basestation Node...\n')
    global node
    node.destroy_node()
    rclpy.shutdown()
    # barometer.close()
    print('\nTerminating Program...\n')
    os.kill(current_pid, signal.SIGTERM)

# Check Wifi
def check_wifi_ssid():
    output = subprocess.check_output(["iwconfig", "2>/dev/null | grep 'ESSID:' | cut -d '\"' -f 2"], shell=True)
    ssid = output.decode('utf-8').strip()
    if(ssid.find("COREBlimp") == -1 and ssid.find("COREBlimp_5G_1") == -1 and ssid.find("COREBlimp_5G_2") == -1):
        print("Invalid WiFi selected! Must be on COREBlimp")
        return False
    else:
        return True

if __name__ == '__main__':
    
    # if(check_wifi_ssid()):
        # Create init function for the following values
        # Initialize default value i.e. goal color value (default: 0)
        # Could make these read from a text file to make them permanent profiles
        global blimps
        blimps = {}

        global auto_panic
        auto_panic = False

        global all_goal_color
        all_goal_color = False

        # Terminate if Ctrl+C Caught
        signal.signal(signal.SIGINT, terminate)

        ros_thread = threading.Thread(target=ros_thread)
        ros_thread.start()

        socketio.run(app, host='192.168.0.200', port=5000)

