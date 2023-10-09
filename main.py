#!/usr/bin/env python3

# Flask Packages
from flask import Flask, render_template, request, Response
from flask_socketio import SocketIO

# ROS Packages
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Int64, Bool, Float64, Float64MultiArray
from yolo_msgs.msg import BoundingBox

# Livestream Packages
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Other Packages
import threading
import time
import sys
import signal
import numpy as np
import json
import subprocess

# Blimp Class
from blimp import Blimp

# Initialize Flask App and SocketIO
app = Flask(__name__)
socketio = SocketIO(app)

class Basestation(Node):

    def __init__(self):
        super().__init__('Basestation')

        # Number of Blimps
        self.numBlimps = 0

        # Recognized Blimp Nodes
        self.recognizedBlimpNodes = []

        # Recognized Blimp Nodes
        self.currentBlimps = []

        # Blimp Node Handlers
        self.blimpNodeHandlers = []

        # Create timer
        self.loopSpeed = 10
        timer_period = 1.0/self.loopSpeed
        self.timer = self.create_timer(timer_period, self.timerLoop)
        self.timeout = 6
        
        # Identify Topic for Teensy to Check if Basestation is On
        self.topicName_identify = "identify"
    
    # This function is not working as intended !!!
    def connectBlimp(self, blimpNodeHandler):

        if blimpNodeHandler.blimpID is not None:
                # Increase Number of New Blimps
                self.numBlimps += 1

                # New Blimp Identified
                self.get_logger().info("Identified new blimp (id: %s). Node name: %s" % (str(blimpNodeHandler.blimpID), blimpNodeHandler.nodeName))
        else:
                # Blimp Altimer_period = 1.0/self.loopSpeedeady Identified
                self.get_logger().info("BlimpID already identified: %s" % blimpNodeHandler.nodeName)
    
    def updateBlimpNodeHandlers(self):
        # Testing
        # print(self.recognizedBlimpNodes)
        # print(self.currentBlimps)
        # print("Number of Blimps:", self.numBlimps)

        # Iterate through current nodes, look for new nodes
        infos = self.get_subscriptions_info_by_topic(self.topicName_identify)
        for info in infos:
            nodeName = info.node_name
            # Testing
            # self.get_logger().info('Node Name: "%s"' % nodeName)
            if nodeName not in self.currentBlimps:
                # Testing
                # self.get_logger().info('Node Name: "%s"' % nodeName)
                if self.check_node_name(nodeName) == True:
                    self.createBlimpNodeHandler(nodeName)

        # Stress Test This !!!

        # Check for nodes that timed out
        for blimpNodeHandler in self.blimpNodeHandlers:
            # No Blimp ID Received
            # if blimpNodeHandler.blimpID is None:
            #     # Check if Blimp Timed Out
            #     if self.getElapsedTime(blimpNodeHandler.timeCreated) > self.timeout:
            #         self.removeBlimpNodeHandler(blimpNodeHandler)
            # elif blimpNodeHandler.lastReceived_blimpID is None:
            #     # Check if Blimp Timed Out
            #     if self.getElapsedTime(blimpNodeHandler.timeCreated) > self.timeout:
            #         self.removeBlimpNodeHandler(blimpNodeHandler)
            # Blimp ID Received
            # else:
            # Remove Blimp Node Handler from List
            if blimpNodeHandler.blimpID is not None:
                if blimpNodeHandler.blimpID not in self.currentBlimps:        
                    self.blimpNodeHandlers.remove(blimpNodeHandler)
                    del blimpNodeHandler
                    continue

            if blimpNodeHandler.blimpID is not None:
                if blimpNodeHandler.lastReceived_blimpID is not None:
                    # print(blimpNodeHandler.blimpID + ": " + str(self.getElapsedTime(blimpNodeHandler.lastReceived_blimpID)))
                    # Check if Blimp Timed Out
                    if self.getElapsedTime(blimpNodeHandler.lastReceived_blimpID) > self.timeout:
                        self.removeBlimpNodeHandler(blimpNodeHandler)
    
    # Stress Test This !!!
    def createBlimpNodeHandler(self, blimp_node_name):
        # Unknown Blimp Node Found
        if blimp_node_name == "_NODE_NAME_UNKNOWN_":
            self.get_logger().info("FLAG: Node Name Unknown")
            return
        
        # Create New Blimp Node Handler
        newBlimpNodeHandler = BlimpNodeHandler(self, blimp_node_name)

        # Check for Blimp ID Topic
        topic_blimpID = "/" + blimp_node_name + "/blimpID"

        # Subscribe to the Blimp ID Topic
        newBlimpNodeHandler.sub_blimpID = self.create_subscription(String, topic_blimpID, newBlimpNodeHandler.listener_callback, 10)

        # Check for State Machine Topic
        topic_state_machine = "/" + blimp_node_name + "/state_machine"

        # Subscribe to the State Machine Topic
        newBlimpNodeHandler.sub_state_machine = self.create_subscription(Int64, topic_state_machine, newBlimpNodeHandler.state_machine_callback, 10)

        # Check for Image Raw Topic
        topic_image_raw = "/" + blimp_node_name + "/left/image_raw"

        # Subscribe to the Image Raw Topic
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        newBlimpNodeHandler.sub_image_raw = self.create_subscription(Image, topic_image_raw, newBlimpNodeHandler.image_raw_callback, qos_profile)

        # Check for State Machine Topic
        topic_bounding_box = "/" + blimp_node_name + "/bounding_box"

        # Subscribe to the State Machine Topic
        newBlimpNodeHandler.sub_state_machine = self.create_subscription(BoundingBox, topic_bounding_box, newBlimpNodeHandler.bounding_box_callback, 10)

        # Add Blimp Node Handler to List
        self.blimpNodeHandlers.append(newBlimpNodeHandler)
        
        # Check if blimp is new or not
        # self.connectBlimp(newBlimpNodeHandler)

        # Add New Blimp Node to Recognized Blimp Nodes
        self.recognizedBlimpNodes.append(blimp_node_name)

    # Stress Test This !!!
    # Fix this function !!!
    def removeBlimpNodeHandler(self, blimpNodeHandler):
        # Timeout Detected for Blimp Node
        if blimpNodeHandler is not None:
            if blimpNodeHandler.blimpID is not None:
                if blimpNodeHandler.blimpID in self.currentBlimps:
                    if blimpNodeHandler.blimpID in self.currentBlimps:
                        self.get_logger().info('Detected timeout of blimp ID "%s"' % blimpNodeHandler.blimpID)
                        self.numBlimps -= 1
                        socketio.emit('remove', blimpNodeHandler.blimp_name)
                        self.currentBlimps.remove(blimpNodeHandler.blimpID)
                    del blimpNodeHandler

    # Check if Node Name is valid
    def check_node_name(self, nodeName):
        if nodeName == 'BurnCreamBlimp' or nodeName == 'SillyAhBlimp' or nodeName == 'TurboBlimp' or nodeName == 'GameChamberBlimp' or nodeName == 'FiveGuysBlimp' or nodeName == 'Catch2' or nodeName == 'Catch1' or nodeName == 'Attack1' or nodeName == 'Attack2':
            return True
        else:
            return False

    # Timer Functions #
    
    def timerLoop(self):
        # Update Blimp Nodes
        self.updateBlimpNodeHandlers()

        # Currently not being used
        # Publish Node Topics
        for blimpNodeHandler in self.blimpNodeHandlers:
            blimpNodeHandler.publish()

    def getElapsedTime(self, prevTime):
        elapsedTime = self.get_clock().now() - prevTime
        elapsedTimeSec = elapsedTime.nanoseconds / 10**9
        return elapsedTimeSec

class BlimpNodeHandler:
    def __init__(self, parentNode, nodeName):
        self.parentNode = parentNode
        self.nodeName = nodeName
        self.timeCreated = self.parentNode.get_clock().now()

        self.blimpID = None
        self.lastReceived_blimpID = None
        self.blimp_name = None
        self.identified_count = 0

        self.pub_auto = None
        self.pub_goal_color = None
        self.pub_killed = None
        self.pub_motorCommands = None
        self.pub_grabbing = None
        self.pub_shooting = None
        # self.pub_base_barometer = None

        # Livestream Most Recent Frame
        self.frame = None
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Check blimp ID and give it a name on the Basestation
        # Make the following if statement a function !!!
        global blimps
        if self.blimpID is None:
            self.blimpID = msg.data
            #self.parentNode.get_logger().info('Identified new blimp with ID "%s"' % msg.data)
            #self.parentNode.numBlimps += 1
            #self.parentNode.connectBlimp(self)
            self.createPublishers()
            self.blimp_name = self.get_blimp_name()
            if self.blimp_name not in blimps:
                self.parentNode.get_logger().info('Identified Blimp with ID "%s"' % msg.data)
                self.parentNode.numBlimps += 1
                blimp = Blimp(self.blimp_name)
                self.get_blimp_type(blimp)
                blimps[self.blimp_name] = blimp
            else:
                self.parentNode.get_logger().info("BlimpID already identified: %s" % msg.data)
                self.parentNode.numBlimps += 1
                # Not working
                #self.parentNode.get_logger().info("%i" % self.parentNode.numBlimps)
        else:
            if self.blimpID not in self.parentNode.currentBlimps:
                self.parentNode.currentBlimps.append(self.blimpID)
            self.lastReceived_blimpID = self.parentNode.get_clock().now()
        
        # Do we still need a heartbeat ???
        # if blimp is not None:
            # blimp.lastHeartbeatDetected = time.time()

        # Emit the blimp data to the webpage
        socketio.emit('update', blimps[self.blimp_name].to_dict())
    
        # Publish the target and/or goal color values to ROS
        # Make this independent of the listener callback !!!
        self.publish_target_color()
        self.publish_goal_color()
        self.publish_motorCommands()
        self.publish_auto()
        self.publish_grabbing()
        self.publish_shooting()

    # Continually Poll State Machine Data from Teensy
    def state_machine_callback(self, msg):
        global blimps
        if self.blimp_name is not None:
            if self.blimp_name in blimps:
                if self.blimpID is not None:
                    blimps[self.blimp_name].state_machine = msg.data

    # Continually Poll Image Raw Data from Pi
    def image_raw_callback(self, msg):
        global blimps
        if self.blimp_name is not None:
            if self.blimp_name in blimps:
                if self.blimpID is not None:
                    try:
                        # Convert the ROS Image message to a CV2 image (numpy ndarray)
                        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                        self.frame = cv_image

                        if self.frame is not None:
                            flag, jpeg = cv2.imencode('.jpg', self.frame)
                            blimps[self.blimp_name].frame = jpeg
                        
                        # Uncomment the following line if you want to see the image using OpenCV
                        # cv2.imshow("Received Image", cv_image)
                        # cv2.waitKey(1)

                    except Exception as e:
                        self.parentNode.get_logger().error(f"Failed to convert image: {e}")

    # Continually Poll State Machine Data from Teensy
    def bounding_box_callback(self, msg):
        global blimps
        bb_dict = self.bounding_box_to_dict(msg)
        if self.blimp_name is not None:
            if self.blimp_name in blimps:
                if self.blimpID is not None:
                    if bb_dict is not None:
                        blimps[self.blimp_name].bounding_box = bb_dict
                        # Emit the bounding box data to the webpage
                        socketio.emit('bounding_box', blimps[self.blimp_name].bounding_box)

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
        if self.blimpID == 'Attack1':
            blimp.blimp_type = 1
        elif self.blimpID == 'Attack2':
            blimp.blimp_type = 1

    def get_blimp_name(self):
        # Blimp Name not Recognized (This should not happen!)
        self.blimp_name = 'Error'

        # Real Blimps #

        # Catching Blimps
        if self.blimpID == 'BurnCreamBlimp':
            self.blimp_name = 'Burn Cream Blimp'
        elif self.blimpID == 'SillyAhBlimp':
            self.blimp_name = 'Silly Ah Blimp'
        elif self.blimpID == 'TurboBlimp':
            self.blimp_name = 'Turbo Blimp'
        elif self.blimpID == 'GameChamberBlimp':
            self.blimp_name = 'Game Chamber Blimp'
        elif self.blimpID == 'FiveGuysBlimp':
            self.blimp_name = 'Five Guys Blimp'   

        # Attacking Blimps   
        # None so far...    

        # Fake Blimps #

        # Catching Blimps
        elif self.blimpID == 'Catch2':
            self.blimp_name = 'Catch 2'
        elif self.blimpID == 'Catch1':
            self.blimp_name = 'Catch 1'

        # Attacking Blimps
        elif self.blimpID == 'Attack1':
            self.blimp_name = 'Attack 1'
        elif self.blimpID == 'Attack2':
            self.blimp_name = 'Attack 2'

        return self.blimp_name

    def publish_target_color(self):
        global blimps
        # Publish goal color value to the ROS topic
        msg = Int64()
        msg.data = blimps[self.blimp_name].target_color
        self.pub_target_color.publish(msg)

    def publish_goal_color(self):
        global blimps
        # Publish goal color value to the ROS topic
        msg = Int64()
        msg.data = blimps[self.blimp_name].goal_color
        self.pub_goal_color.publish(msg)

    def publish_motorCommands(self):
        global blimps
        # Publish motor commands value to the ROS topic
        msg = Float64MultiArray()
        msg.data = blimps[self.blimp_name].motorCommands
        self.pub_motorCommands.publish(msg)

    def publish_auto(self):
        global blimps
        # Publish auto value to the ROS topic
        msg = Bool()
        msg.data = blimps[self.blimp_name].auto
        self.pub_auto.publish(msg)

    def publish_grabbing(self):
        global blimps
        # Publish grabbing value to the ROS topic
        msg = Bool()
        msg.data = blimps[self.blimp_name].grabbing
        self.pub_grabbing.publish(msg)

    def publish_shooting(self):
        global blimps
        # Publish shooting value to the ROS topic
        msg = Bool()
        msg.data = blimps[self.blimp_name].shooting
        self.pub_shooting.publish(msg)

    # Update Blimp Class with Dictionary Data
    @socketio.on('update_blimp_dict')
    def update_blimp_dict(data):
        global blimps
        blimp_name = data['blimp_name']
        blimps[blimp_name] = data[blimp_name]

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
        blimp_name = data['blimp_name']
        blimps[blimp_name].target_color = data['target_color']

    # Update Goal Color
    @socketio.on('update_goal_color')
    def update_goal_color(data):
        global blimps
        blimp_name = data['blimp_name']
        blimps[blimp_name].goal_color = data['goal_color']

    def createPublishers(self):
        topic_auto =            "/" + self.nodeName + "/auto"
        topic_goal_color =      "/" + self.nodeName + "/goal_color"
        topic_target_color =      "/" + self.nodeName + "/target_color"
        topic_killed =          "/" + self.nodeName + "/killed"
        topic_motorCommands =  "/" + self.nodeName + "/motorCommands"
        topic_grabbing =        "/" + self.nodeName + "/grabbing"
        topic_shooting =        "/" + self.nodeName + "/shooting"
        # topic_base_barometer =   "/" + self.nodeName + "/base_barometer"

        bufferSize = 1
        self.pub_auto = self.parentNode.create_publisher(Bool, topic_auto, bufferSize)
        self.pub_goal_color = self.parentNode.create_publisher(Int64, topic_goal_color, bufferSize)
        self.pub_target_color = self.parentNode.create_publisher(Int64, topic_target_color, bufferSize)
        self.pub_killed = self.parentNode.create_publisher(Bool, topic_killed, bufferSize)
        self.pub_motorCommands = self.parentNode.create_publisher(Float64MultiArray, topic_motorCommands, bufferSize)
        self.pub_grabbing = self.parentNode.create_publisher(Bool, topic_grabbing, bufferSize)
        self.pub_shooting = self.parentNode.create_publisher(Bool, topic_shooting, bufferSize)
        # self.pub_base_barometer = self.parentNode.create_publisher(Float64, topic_base_barometer, bufferSize)

    def publish(self):
        pass
        """
        if self.blimp is None:
            return
        
        msg_auto = Bool(data=self.blimp.auto)
        msg_goal_color = Bool(data=self.blimp.auto)
        msg_killed = Bool(data=self.blimp.killed)
        msg_motorCommands = Float64MultiArray(data=self.blimp.motorCommands)
        msg_grabbing = Bool(data=self.blimp.grabbing)
        msg_shooting = Bool(data=self.blimp.shooting)
        msg_baseBarometer = Float64(data=self.blimp.baseBarometer)

        self.pub_auto.publish(msg_auto)
        self.pub_goal_color.publish(msg_goal_color)
        self.pub_killed.publish(msg_killed)
        self.pub_motorCommands.publish(msg_motorCommands)
        self.pub_grabbing.publish(msg_grabbing)
        self.pub_shooting.publish(msg_shooting)
        self.pub_baseBarometer.publish(msg_baseBarometer)
        """

# Handle user connection to webpage
@socketio.on('connect')
def handle_connect():
    print('Client connected with IP:', request.remote_addr)

#Main Basestation Page
@app.route('/')
def index():
    client_ip = request.remote_addr
    return render_template('main.html', client_ip=client_ip)

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

# Streaming Endpoints
@app.route('/Burn Cream Blimp')
def burnCreamBlimpPage():
    return render_template('Burn Cream Blimp.html')

@app.route('/Silly Ah Blimp')
def sillyAhhBlimpPage():
    return render_template('Silly Ah Blimp.html')

@app.route('/Turbo Blimp')
def turboBlimpPage():
    return render_template('Turbo Blimp.html')  

@app.route('/Game Chamber Blimp')
def gameChamberBlimpPage():
    return render_template('Game Chamber Blimp.html')

@app.route('/Five Guys Blimp')
def fiveGuysBlimpPage():
    return render_template('Five Guys Blimp.html')

@app.route('/Catch 1')
def catch1Page():
    return render_template('Catch 1.html')

@app.route('/Catch 2')
def catch2Page():
    return render_template('Catch 2.html')

@app.route('/Attack 1')
def attack1Page():
    return render_template('Attack 1.html')

@app.route('/Attack 2')
def attack2Page():
    return render_template('Attack 2.html')

def ros_thread():
    rclpy.init()

    global node
    node = Basestation()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

def terminate(signal, frame):
    print('\nTerminating...\n')
    global node
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

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

        # Terminate if Ctrl+C Caught
        signal.signal(signal.SIGINT, terminate)

        ros_thread = threading.Thread(target=ros_thread)
        ros_thread.start()

        socketio.run(app, host='192.168.0.200', port=5000)

