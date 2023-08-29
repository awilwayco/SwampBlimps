#!/usr/bin/env python3

# Flask Packages
from flask import Flask, render_template, request
from flask_socketio import SocketIO

# ROS Packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64, Bool, Float64, Float64MultiArray

# Other Packages
import threading
import time

# Initialize Flask App and SocketIO
app = Flask(__name__)
socketio = SocketIO(app)

class Basestation(Node):

    def __init__(self):
        super().__init__('Basestation')

        # Number of Blimps
        self.numNewBlimps = 0

        # Recognized Blimp Nodes
        self.recognizedBlimpNodes = []

        # Blimp Node Handlers
        self.blimpNodeHandlers = []

        # Create timer
        self.loopSpeed = 0.1
        timer_period = 1.0/self.loopSpeed
        self.timer = self.create_timer(timer_period, self.timerLoop)
        self.timeout = 1
        
        # Identify Topic for Teensy to Check if Basestation is On
        self.topicName_identify = "identify"

        # Need to scale for unlimited # of Blimps for Subscriptions and Publishers
        
        # Subscribed Values (Blimp IDs and state_machine)
        # self.subscription = self.create_subscription(String, 'WaffleBlimp/blimpID', self.listener_callback, 10)
        # self.subscription = self.create_subscription(String, 'BurnCreamBlimp/blimpID', self.listener_callback, 10)
        # self.subscription = self.create_subscription(String, 'Blimp2/blimpID', self.listener_callback, 10)
        # self.subscription = self.create_subscription(String, 'Blimp1/blimpID', self.listener_callback, 10)
        # Published Values (Goal Color for Catching Blimps, Target Color for Attacking Blimps, and Auto)
        # self.publisher_goal_color = self.create_publisher(Int64, 'Blimp2/goal_color', 10) 
        # self.publisher_goal_color = self.create_publisher(Int64, 'Blimp1/goal_color', 10) 
    
    def connectBlimp(self, blimpNodeHandler):

        if blimpNodeHandler.nodeName not in self.recognizedBlimpNodes:
            # Increase Number of New Blimps
            self.numNewBlimps += 1

            # New Blimp Identified
            self.get_logger().info("Identified new blimp (id: %s). Node name: %s" % (str(blimpNodeHandler.blimpID), blimpNodeHandler.nodeName))

        else:
            # Blimp Already Identified
            self.get_logger().info("BlimpID already identified: %s" % blimpNodeHandler.nodeName)
    

    def updateBlimpNodeHandlers(self):
        # Testing
        print(self.recognizedBlimpNodes)
        # Iterate through current nodes, look for new nodes
        infos = self.get_subscriptions_info_by_topic(self.topicName_identify)
        for info in infos:
            nodeName = info.node_name
            # Testing
            # self.get_logger().info('Node Name: "%s"' % nodeName)
            if nodeName not in self.recognizedBlimpNodes:
                # self.get_logger().info('Node Name: "%s"' % nodeName)
                self.createBlimpNodeHandler(nodeName)

        """
        # Print out names of all connected nodes
        connectedNodes = ""
        for node in self.recognizedNodes:
            connectedNodes += node + ", "
        self.get_logger().info("Connected nodes: %s" % connectedNodes)
        """

        # Check for nodes that timed out
        
        for blimpNodeHandler in self.blimpNodeHandlers:

            # No Blimp ID Received
            if blimpNodeHandler.lastReceived_blimpID is None:
                # Check if Blimp Timed Out
                if self.getElapsedTime(blimpNodeHandler.timeCreated) > self.timeout:
                    self.removeBlimpNodeHandler(blimpNodeHandler)

            # Blimp ID Received
            else:

                # Double Check if Blimp Timed Out
                if self.getElapsedTime(blimpNodeHandler.lastReceived_blimpID) > self.timeout:
                    self.removeBlimpNodeHandler(blimpNodeHandler)

    def createBlimpNodeHandler(self, blimp_node_name):
        # Unknown Blimp Node Found
        if blimp_node_name == "_NODE_NAME_UNKNOWN_":
            self.get_logger().info("FLAG: Node Name Unknown")
            return

        global goal_colors
        goal_colors[blimp_node_name] = 0

        # Add New Blimp Node to Recognized Blimp Nodes
        self.recognizedBlimpNodes.append(blimp_node_name)

        # Create New Blimp Node Handler
        newBlimpNodeHandler = BlimpNodeHandler(self, blimp_node_name)

        # Check for Blimp ID Topic
        topic_blimpID = "/" + blimp_node_name + "/blimpID"

        # Subscribe to the Blimp ID Topic
        newBlimpNodeHandler.sub_blimpID = self.create_subscription(String, topic_blimpID, newBlimpNodeHandler.listener_callback, 10)

        # Add Blimp Node Handler to List
        self.blimpNodeHandlers.append(newBlimpNodeHandler)

    # Fix this function eventually when New Method is Implemented !!!
    def removeBlimpNodeHandler(self, blimpNodeHandler):
        # Remove Blime Node Name from List
        self.recognizedBlimpNodes.remove(blimpNodeHandler.nodeName)

        # Remove Blimp Node Handler from List
        self.blimpNodeHandlers.remove(blimpNodeHandler)

        # Timeout Detected for Blimp Node
        if blimpNodeHandler is not None:
            self.get_logger().info('Detected timeout of blimp ID "%s"' % blimpNodeHandler.blimpID)

    # Timer Functions #
    
    def timerLoop(self):
        # Update Blimp Nodes
        self.updateBlimpNodeHandlers()

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
        # self.parentNode.blimpNodeHandlers[self] = self

        self.blimpID = None
        self.lastReceived_blimpID = None
        self.blimp = None
        self.goal_color = 0

        self.pub_auto = None
        self.pub_goal_color = self.goal_color
        self.pub_killed = None
        self.pub_motorCommands = None
        self.pub_grabbing = None
        self.pub_shooting = None
        self.pub_baseBarometer = None

    def listener_callback(self, msg):
        # Check blimp ID and give it a name on the Basestation
        # Make a function for hard-coding blimp names

        if self.blimpID is None:
            self.blimpID = msg.data
            self.parentNode.get_logger().info('Identified new blimp with ID "%s"' % msg.data)
            self.parentNode.connectBlimp(self)
            self.createPublishers()
        self.lastReceived_blimpID = self.parentNode.get_clock().now()
        if self.blimp is not None:
            self.blimp.lastHeartbeatDetected = time.time()

        blimp_name = self.get_blimp_name(msg.data)

        # self.get_logger().info('Publishing: "%s"' % msg.data)

        # Emit the blimp names to the webpage
        socketio.emit('blimp_name', {'node_name': self.nodeName, 'blimp_name': blimp_name})
        # Testing
        # global goal_colors
        # socketio.emit('update', {'node_name': self.nodeName, 'blimp_name': blimp_name, 'goal_color_data': goal_colors[self.nodeName]})

        # Emit the goal color value to the webpage
        self.emit_goal_color()

    def get_blimp_name(self, blimp_ID):
        blimp_name = 'Error'

        if blimp_ID == 'WaffleBlimp':
            blimp_name = 'Waffle Blimp'
        elif blimp_ID == 'BurnCreamBlimp':
            blimp_name = 'Burn Cream Blimp'
        elif blimp_ID == 'Blimp2':
            blimp_name = 'Blimp 2'
        elif blimp_ID == 'Blimp1':
            blimp_name = 'Blimp 1'

        return blimp_name

    def emit_goal_color(self):

        # Emit the goal color to the webpage
        global goal_colors
        socketio.emit('goal_color', {'node_name': self.nodeName, 'goal_color': goal_colors[self.nodeName]})

        # Publish goal color value to the ROS topic
        msg = Int64()
        msg.data = goal_colors[self.nodeName]
        self.pub_goal_color.publish(msg)

    # Update Goal Color
    @socketio.on('update_goal_color')
    def update_goal_color(data):
        node_name = data['node_name']
        goal_color_data = data['goal_color_data']
        
        global goal_colors
        goal_colors[node_name] = goal_color_data
        print(goal_colors[node_name])

    def createPublishers(self):
        topic_auto =            "/" + self.nodeName + "/auto"
        topic_goal_color =            "/" + self.nodeName + "/goal_color"
        topic_killed =          "/" + self.nodeName + "/killed"
        topic_motorCommands =   "/" + self.nodeName + "/motorCommands"
        topic_grabbing =        "/" + self.nodeName + "/grabbing"
        topic_shooting =        "/" + self.nodeName + "/shooting"
        topic_baseBarometer =   "/" + self.nodeName + "/baseBarometer"

        bufferSize = 1
        self.pub_auto = self.parentNode.create_publisher(Bool, topic_auto, bufferSize)
        self.pub_goal_color = self.parentNode.create_publisher(Int64, topic_goal_color, bufferSize)
        self.pub_killed = self.parentNode.create_publisher(Bool, topic_killed, bufferSize)
        self.pub_motorCommands = self.parentNode.create_publisher(Float64MultiArray, topic_motorCommands, bufferSize)
        self.pub_grabbing = self.parentNode.create_publisher(Bool, topic_grabbing, bufferSize)
        self.pub_shooting = self.parentNode.create_publisher(Bool, topic_shooting, bufferSize)
        self.pub_baseBarometer = self.parentNode.create_publisher(Float64, topic_baseBarometer, bufferSize)

    def publish(self):
        if self.blimp is None:
            return
        """
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

@app.route('/')
def index():
    return render_template('main.html')

def ros_thread():
    rclpy.init()

    node = Basestation()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    # Create init function for the following values
    # Need to scale these !!!
    # Initialize goal color value (default: 0)
    # Could make these read from a text file to make them permanent profiles
    global goal_colors
    goal_colors = {}

    ros_thread = threading.Thread(target=ros_thread)
    ros_thread.start()

    socketio.run(app, host='0.0.0.0', port=5000)
