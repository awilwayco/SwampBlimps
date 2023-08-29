# Imported Classes
from Blimp import Blimp
from InputHandler import InputHandler
from UDPMulticast import UDPHelper
from BlimpMapper import BlimpMapper

# Imported Libraries
import time
import easygui
import serial

# Not used #
# from SerialHelper import SerialHelper


# ======================== Blimp Handler Class ======================== #
class BlimpHandler:

# =================== Blimp Handler Class Functions =================== #

    # ================ Initialization of Blimp Handler ================ #
    # Description #
    # Sets up UDP and initializes variables to default values
    def __init__(self):
        # Communication Initialization (UDP)
        self.comms = UDPHelper()  # SerialHelper() # UDPHelper()

        # Display Initialization
        self.display = None

        # Open communication (UDP)
        self.comms.open()

        # List of Blimps
        self.blimps = []

        # Initialize dict storing indices of blimps
        self.blimpIndexMap = {}

        # Blimp Names and IPs
        self.blimpIPNameMap = {"192.168.0.101": "Spicy Hot Dog",
                               "192.168.0.102": "Waffle",
                               "192.168.0.103": "Apple",
                               "192.168.0.104": "Milk",
                               "192.168.0.105": "Pasta",

                               "192,168.0.100": "Silly Ahh",

                               "192.168.0.80": "Big Cup of Eggs",
                               "192.168.0.20": "Leg in a Cup",
                               "192.168.0.89": "I'm in a Cup",
                               "192.168.0.62": "My Cup of Eggs",
                               "192.168.0.86": "Pint of Eggs",
                               "192.168.0.14": "Stealthy Steve",

                               "192.168.0.38": "Barometer"}

        #Map of established connections (i.e. EC)
        self.blimpECMap = {"192.168.0.101": False,
                            "192.168.0.102": False,
                            "192.168.0.103": False,
                            "192.168.0.104": False,
                            "192.168.0.105": False,

                            "192.168.0.80": False,
                            "192.168.0.20": False,
                            "192.168.0.89": False,
                            "192.168.0.62": False,
                            "192.168.0.86": False,
                            "192.168.0.14": False,

                            "192.168.0.38": False}

        #Hard-coded list of all blimps
        self.swampBlimps = {}

        for key in self.blimpIPNameMap:
            newBlimp = Blimp(key, self.blimpIPNameMap[key])
            self.swampBlimps[key] = newBlimp

        self.numNewBlimps = 0

        # Initialize Input Handler
        self.inputHandler = InputHandler()

        # Initialize Blimp Mapper
        self.blimpMapper = BlimpMapper(self, self.inputHandler)

        # Blimp States
        self.blimpStateStrings = {-1: "None",
                                  0: "searching",
                                  1: "approach",
                                  2: "catching",
                                  3: "caught",
                                  4: "goalSearch",
                                  5: "approachGoal",
                                  6: "scoringStart",
                                  7: "shooting",
                                  8: "scored"}

        self.lastBlimpAdded = 0
        self.blimpAddDelay = 5

        # Set Base Height
        self.baseHeight = 0

        # Barometer Initialization
        self.baroPrioritizeUDP = False
        self.baroTimeout = 3  # seconds
        self.baroType = None
        self.lastBaroPrint = 0
        self.baroUDPLastReceivedTime = 0
        self.baroUDPLastReceivedValue = None
        self.baroSerialLastReceivedTime = 0
        self.baroSerialLastReceivedValue = None
        self.baroSerial = None

        # Try connecting to the Barometer
        try:
            self.baroSerial = serial.Serial('/dev/baro_0', 115200)
        except(serial.SerialException):
            # Error
            print("Serial error!")

        # Plot Data
        self.plotData = {}

        # Initialize Total Number of Messages
        self.numMessages = 0

        # Initialize Last Checked Number of Messages
        self.lastCheckedNumMessages = 0

        # Initialize Last Loop Update
        self.lastUpdateLoop = 0

        # Initialize Global Targets
        self.globalTargets = False

        #self.addFakeBlimp(1000,"Fake Blimp 1000")

    # ====================== Close Blimp Handler ====================== #
    # Description #
    # Closes UDP
    def close(self):
        # Print Closing Message
        print("Closing BlimpHandler")

        # Close Communication (UDP)
        self.comms.close()

        # Print All Comms Closed
        print("Comms closed.")

    # ================== Set Display of Blimp Handler ================= #
    # Description #
    # Sets up the display and shows controllers
    def setDisplay(self, display):
        # Set Display
        self.display = display

    # ================== Update Blimp Handler (Loop) ================== #
    # Description #
    # Loops to continuously check and update barometer height, dead blimps,
    # general blimp data, and wait time since last update
    def update(self):
        # Update Barometer Height
        self.updateBaroHeight()
        # Update Input Handler
        self.inputHandler.update()
        # Check for Dead Blimps
        self.checkForDeadBlimps()
        # Update Blimp Mapper
        self.blimpMapper.update()
        # Get Blimp Data
        self.listen()
        # Update Blimp Data
        self.sendDataToBlimps()

        # Calculate and Print Wait Time
        waitTime = time.time() - self.lastUpdateLoop
        if (waitTime > 0.01):
            print(waitTime)

        # Set Last Loop Update to current time
        self.lastUpdateLoop = time.time()

    # ====================== Update Baro Height ======================= #
    # Description #
    # Gets Height of Barometer and Validates the Data
    # *Note*
    # Make UI element that shows if barometer data is available and what
    # type such as Serial or UDP
    def updateBaroHeight(self):
        # Handle Barometer Serial Connection and Get Height
        self.handleBaroSerial()

        # Validate Recent Barometer Data
        self.validateBaroDataReceived()

    # ===================== Check for Dead Blimps ===================== #
    # Description #
    # Finds disconnected blimps and removes them from the Blimp List
    def checkForDeadBlimps(self):
        # Iterate through blimps
        for ID in self.swampBlimps.keys():
            blimp = self.swampBlimps[ID]
            if not blimp.connected:
                continue
            # Calculate time since blimp's last heartbeat
            blimp.lastHeartbeatDiff = time.time() - blimp.lastHeartbeatDetected
            if blimp.lastHeartbeatDiff > blimp.heartbeatDisconnectDelay:
                # Blimp heartbeat not received for too long; Remove it
                print(blimp.name, "heartbeat not received; Disconnecting...")
                self.removeBlimp(blimp.ID)
                blimp.connected = False
                if ID in self.blimpECMap:
                    self.blimpECMap[ID] = False


    #========================== Remove Blimp ========================== #
    # Description #
    # If ID is an identified blimp, disconnects the blimp
    def removeBlimp(self, ID):
        # Check that valid ID is passed in
        if ID in self.blimpECMap:
            self.blimpECMap[ID] = False

    # ====================== Listen for Messages ====================== #
    # Description #
    # Prints last checked number of messages, then gets current number of messages
    def listen(self):
        # Print Current Number of Messages
        if (time.time() - self.lastCheckedNumMessages > 1):
            self.lastCheckedNumMessages = time.time()
            print("NumMessages:",self.numMessages)
            self.numMessages = 0

        # Get Current Number of Messages
        readStrings = self.comms.getInputMessages()
        # readStrings = []
        while(len(readStrings)>0):
            self.numMessages += 1
            for readString in readStrings:
                pass
                #print(readString)
            message = readStrings[0]
            readStrings.pop(0)
            #self.checkForNewBlimps(message)
            self.useMessage(message) # includes heartbeat

    # ====================== Send Data to Blimps ====================== #
    # Description #
    # Sends Blimp ID, Message Data, Inputs
    def sendDataToBlimps(self):
        # Get Current Time
        currentTime = time.time()

        # Iterate through inputs
        for input in self.inputHandler.inputs:
            # Iterate through blimps mapped to this input
            mappedBlimp = self.blimpMapper.getMappedBlimp(input.name)
            if mappedBlimp is not None:
                # Send blimp inputs
                self.sendBlimpInputs(input, mappedBlimp, currentTime)
                # Send blimp-centered actions
                self.blimpCenteredActions(input, mappedBlimp)

            # Non-blimp-centered Actions
            self.nonBlimpCenteredActions(input)

    # ======================= Request Recording ======================= #
    # Description #
    # Makes sure Blimp IDs is a list and sends Blimp ID over UDP
    def requestRecording(self, blimpIDs):
        if (type(blimpIDs) != list):
            blimpIDs = [blimpIDs]
        for blimpID in blimpIDs:
            self.comms.send(blimpID, "P", "C300")

# ========================= Helper Functions ========================== #

    # ======================= Send Blimp Inputs ======================= #
    # Description #
    # Gets inputs and sends to blimp
    # with Blimp Data (Blimp Height, Target Goal, and Target Enemy)
    def sendBlimpInputs(self, input, blimp, currentTime):
        inputData = input.grabInput()
        blimpID = blimp.ID

        # If the time since previous input is GREATER than the time input delay
        if currentTime - blimp.lastTimeInputDataSent > blimp.timeInputDelay:
            # Store the current time
            blimp.lastTimeInputDataSent = currentTime
            # If the blimp is autonomous
            if blimp.auto == 1:
                # Send Blimp ID and a message about base height, target goal, and target enemy (UDP)
                message = str(self.baseHeight) + ";" + blimp.targetGoal + ";" + blimp.targetEnemy
                self.comms.send(blimpID, "A", message)
            # If the time since previous input is LESS than the time input delay
            else:
                # Set a copy of Input Data to Blimp Data
                blimpData = inputData.copy()
                # Add Grabbing and Shooting to Blimp Data
                blimpData.append(blimp.grabbing)
                blimpData.append(blimp.shooting)
                message = ""
                # Iterate through all Blimp Data to add to the message
                for data in blimpData:
                    message += str(data) + ","
                # Send Blimp ID and a message about base height, target goal, and target enemy (UDP)
                message += ";" + str(self.baseHeight) + ";" + blimp.targetGoal + ";" + blimp.targetEnemy
                self.comms.send(blimpID, "M", message)

    # ===================== Blimp Centered Actions ==================== #
    # Description #
    # Changes Blimp's Action based on Input
    def blimpCenteredActions(self, input, blimp):
        blimpID = blimp.ID
        # Grab
        if (input.grabAction("grab")):
            blimp.grabbing = 1 - blimp.grabbing
            print("Toggled grabber for blimp ID", blimpID)

        # Shoot
        if (input.grabAction("shoot")):
            blimp.shooting = 1 - blimp.shooting
            print("Toggled shooting for blimp ID", blimpID)

        # Autonomous
        if (input.grabAction("auto")):
            blimp.auto = 1 - blimp.auto
            print("Toggled auto for blimp ID", blimpID)
            input.notify(0.5)

        # Kill
        if (input.grabAction("kill")):
            print("Killing blimp", blimpID)
            self.comms.send(blimpID, "K", "")
            input.notify(3)

        # Record
        if (input.grabAction("record")):
            self.requestRecording(blimpID)

    # =================== Non-Blimp Centered Actions ================== #
    # Description #
    # Changes Blimp's Action based on Input
    def nonBlimpCenteredActions(self, input):
        # Autonomous Panic
        if input.grabAction("panicAuto"):
            # Iterate through blimps
            for ID in self.swampBlimps.keys():
                blimp = self.swampBlimps[ID]
                blimp.auto = 1
            print("PANIC AUTO")
            input.notify(1)
            self.blimpMapper.clearMappings()

        # Connect Up
        if input.grabAction("connectUp"):
            self.blimpMapper.mapUp(input.name)

        # Connect Down
        if input.grabAction("connectDown"):
            self.blimpMapper.mapDown(input.name)

        # Vibe Right
        if input.grabAction("vibeRight"):
            input.notify(0.1)
            print("STOP Vibe")

        # Vibe Left
        if input.grabAction("vibeLeft"):
            input.notify(100000)
            print("STOP Vibe")

    # =============== Handle Barometer Serial Connection ============== #
    # Description #
    # If Barometer is Disconnected, Attempt to Reconnect
    # If Barometer is Connected, Get Barometer Data from the Teensy
    def handleBaroSerial(self):
        # Barometer is Disconnected
        if (self.baroSerial is None):
            # Try to reconnect
            try:
                self.baroSerial = serial.Serial('/dev/baro_0', 115200)
                self.baroSerial.open()
            # If reconnect fails
            except(serial.SerialException):
                # Nothing happens
                pass

        # Barometer is Connected
        if (self.baroSerial is not None):
            # Get Barometer Data from Teensy
            try:
                while self.baroSerial.in_waiting:
                    receivedString = self.baroSerial.readline().decode('utf-8')
                    if (self.isFloat(receivedString)):
                        self.baroSerialLastReceivedValue = float(receivedString)
                        self.baroSerialLastReceivedTime = time.time()
                if (time.time() - self.lastBaroPrint > 0.01):
                    self.lastBaroPrint = time.time()
                    # print(self.baseHeight)
            # If port crashes, assume teensy has disconnected
            except(OSError):
                self.baroSerial = None

    # ================ Validate Barometer Data Received =============== #
    # Description #
    # Checks Barometer Data and Validates it
    def validateBaroDataReceived(self):
        # Check which data has been received recently
        currentTime = time.time()
        baroValidUDP = currentTime - self.baroUDPLastReceivedTime < self.baroTimeout
        baroValidSerial = currentTime - self.baroSerialLastReceivedTime < self.baroTimeout and self.baroSerial is not None

        if (baroValidUDP and (self.baroPrioritizeUDP or not baroValidSerial)):
            self.baseHeight = self.baroUDPLastReceivedValue
            self.baroType = "UDP"
        elif (baroValidSerial and (not self.baroPrioritizeUDP or not baroValidUDP)):
            self.baseHeight = self.baroSerialLastReceivedValue
            self.baroType = "Serial"
        else:
            self.baseHeight = None
            self.baroType = None

    """ # Deprecated?
    # ========================== Input Exists ========================= #
    # Description #
    # Returns true and index or false and zero for Input Name
    def inputExists(self, inputName):
        for i in range(0,len(self.inputs)):
            if (inputName == self.inputs[i].name):
                return (True, i)
        return (False, 0)
    """

    """ # Deprecated?
    # ======================= Blimp Name Exists ======================= #
    # Description #
    # Returns true and index or false and 0 for Blimp Name
    def blimpNameExists(self, blimpName):
        for i in range(0,len(self.blimps)):
            if (blimpName == self.blimps[i].name):
                return (True, i)
        return (False, 0)
    """

    """ # Deprecated?
    # ======================== Blimp ID Exists ======================== #
    # Description #
    # Returns true or false for Blimp ID
    def blimpIDExists(self, blimpID):
        for blimp in self.blimps:
            if (blimpID == blimp.ID):
                return True
        return False
    """

    """ # Deprecated?
    # =================== Check if New Blimps Exist =================== #
    # Description #
    # Check for New Blimps and Create New Blimp Objects
    def checkForNewBlimps(self, message):
        # Get Message Content and Address
        msgContent = message[0]
        msgAddress = message[1]
        if (msgContent == "0,N:N:N"):  # New Blimp
            # Get Current Time
            currentTime = time.time()
            # If Time since Last Blimp Added is Greater than Blimp Add Delay
            if (currentTime - self.lastBlimpAdded > self.blimpAddDelay):
                self.lastBlimpAdded = currentTime
                # Add new blimp!

                # Check if IP matches known blimp
                #Assumes msgAddress is full IP Address

                #newID = self.blimpIPIDMap.get(msgAddress)

                #If new IP exists in defined blimps
                if msgAddress in self.blimpECMap:
                    print('Recognized Blimp', msgAddress)

                    #This may be messed up now
                    self.comms.send('N','N', str(msgAddress))

                    #Update heartbeat of blimp in blimps list if already added
                    if(self.blimpECMap[msgAddress]):
                        for blimp in self.blimps:
                            if blimp.ID == msgAddress:
                                blimp.lastHeartbeatDetected = time.time()
                                break

                    else:
                        self.swampBlimps[msgAddress].lastHeartbeatDetected = time.time()

                        #Using function addBlimp for secure appending of blimp
                        self.addBlimp(msgAddress)
                else:
                    print('Unrecognized Blimp')
            else:
                comma = msgContent.find(",")
                colon = msgContent.find(":")
                if (comma == -1 or colon == -1):
                    return
                checkID = msgContent[comma + 1:colon]
                # If ID is an Integer
                if (self.isInt(checkID)):
                    checkID = int(checkID)
                    self.addBlimp(checkID)
    """

    """ # Not needed?
    #========================== Add Blimp ========================== #
    # Description #
    # Verifies if IP is known, and if the IP has been added to connected blimps, and appends to connected blimps list
    def addBlimp(self, newIP):
        
        #Check that valid IP is passed in, otherwise return
        if newIP not in self.blimpECMap:
            return

        if not (self.blimpECMap[newIP]):
            #Establish connection
            self.blimpECMap[newIP] = True

            #Ensure blimp has not been added to list of connected blimps
            blimpInList = False
            for blimp in self.blimps:
                if (blimp.ID == newIP):
                    blimpInList = True

                #If the blimp has not been added, then add to the list of connected blimps
                #IP has already been verified to be valid, so should exist in swampBlimps
            if not blimpInList:
                self.blimps.append(self.swampBlimps[newIP])
    """

    # ========================== Use Message ========================== #
    # Description #
    # Use the Messages to Update the Blimp Data and Barometer
    def useMessage(self, message):
        # Get the Message Content and Address
        msgContent = message[0]
        msgAddress = message[1]
        comma = msgContent.find(",")
        colon = msgContent.find(":")
        ID = msgContent[comma+1:colon]
        # Assume ID is "correct", but ID could come from 1 of three types of blimps
        # 1: Identified blimp that is connected; 2: Identified blimp that is not connected; 3: Unidentified blimp

        # Check if blimp is identified
        identified = ID in self.swampBlimps
        if not identified:
            # Not previously identified... identify it!
            self.numNewBlimps += 1
            blimpName = "New Blimp " + str(self.numNewBlimps)
            newBlimp = Blimp(ID, blimpName)
            self.swampBlimps[ID] = newBlimp
            print("Identified new blimp. Assigned name:", blimpName)

        # Now assume blimp is identified
        blimp = self.swampBlimps[ID]
        # Get current time
        currentTime = time.time()
        # Update last heartbeat and connected
        blimp.lastHeartbeatDetected = currentTime
        blimp.connected = True
        # Get Flag from the Message
        secondColon = msgContent.find(":", colon+1)
        flag = msgContent[colon+1:secondColon]

        # If Flag is P
        if flag == "P":
            equal = msgContent.find("=", secondColon+1)
            numFeedbackData = int(msgContent[secondColon+1:equal])
            currentDataLength = len(blimp.data)
            for i in range(currentDataLength, numFeedbackData):
                blimp.data.append(0.0)
            lastComma = equal
            for i in range(0, numFeedbackData):
                nextComma = msgContent.find(",", lastComma+1)
                blimp.data[i] = float(msgContent[lastComma+1:nextComma])
                lastComma = nextComma
        # If Flag is S
        if flag == "S": # State
            # Get the Received State of the Blimp
            blimp.receivedState = int(msgContent[secondColon+1:])
            # print("Received State:",blimp.receivedState)
        # If Flag is BB
        if flag == "BB": # BarometerBaseline
            baroMsg = msgContent[secondColon+1:]
            if (self.isFloat(baroMsg)):
                self.baroUDPLastReceivedTime = currentTime
                self.baroUDPLastReceivedValue = float(baroMsg)
        # If Flag is T
        if flag == "T": # Telemetry
            pass
            """
            msg = msgContent[secondColon+1:]
            msgEqual = msg.find("=")
            varName = msg[0:msgEqual]
            varValue = msg[msgEqual+1:]
            if(self.isFloat(varValue)):
                varValue = float(varValue)
                key = str(blimp.ID) + ":" + varName
                if(self.plotData.get(key) == None):
                    #New variable
                    self.plotData[key] = [[],varValue,varValue] #[Data points, min, max]

                varPlotData = self.plotData.get(key)
                currentData = (time.time(), varValue)
                varPlotData[0].append(currentData)
                varPlotData[1] = min(varPlotData[1],varValue)
                varPlotData[2] = max(varPlotData[2],varValue)
                #print(len(varPlotData[0]))
            """

    # ============================ Push MPB =========================== #
    # Description #
    # Push Message Parameters for Blimps
    # Display If Exclusive Connections are True or False
    # Check if Autonomous is True or False and Update for Each Blimp
    def pushMPB(self, blimpIDs):
        # If BlimpIDs is not a list
        if (type(blimpIDs) != list):
            blimpIDs = (blimpIDs)
        # Update parameter
        data = easygui.enterbox(msg = "Enter parameter data", title = "Parameter Update")
        # If Data doesn't exist, Return
        if (data is None):
            return

    # ==================== Push Target Goal Button ==================== #
    # Description #
    # Change Blimp Target Goal and for Alternate Blimps as well
    # TG = TargetGoal
    def pushTGButton(self, blimpID):
        # Get blimp from ID
        blimp = self.swampBlimps[blimpID]
        oldTargetGoal = blimp.targetGoal
        # Determine new goal based on old goal
        newTargetGoal = None
        if oldTargetGoal == "O":
            newTargetGoal = "Y"
        elif oldTargetGoal == "Y":
            newTargetGoal = "O"
        # Update goals for either just this blimp OR all blimps
        if not self.globalTargets:
            blimp.targetGoal = newTargetGoal
        else:
            for ID in self.swampBlimps.keys():
                currentBlimp = self.swampBlimps[ID]
                currentBlimp.targetGoal = newTargetGoal


    # ==================== Push Target Enemy Button =================== #
    # Description #
    # Change Blimp Target Enemy and for Alternate Blimps as well
    # TE = TargetEnemy
    def pushTEButton(self, ID):
        # Get blimp by ID
        blimp = self.swampBlimps[ID]
        oldTargetEnemy = blimp.targetEnemy
        # Determine new target enemy based on old target enemy
        newTargetEnemy = None
        if oldTargetEnemy == "R":
            newTargetEnemy = "B"
        elif oldTargetEnemy == "B":
            newTargetEnemy = "G"
        elif oldTargetEnemy == "G":
            newTargetEnemy = "R"
        # Update target enemy either just for this blimp or all blimps
        if not self.globalTargets:
            blimp.targetEnemy = newTargetEnemy
        else:
            for currentID in self.swampBlimps.keys():
                currentBlimp = self.swampBlimps[currentID]
                currentBlimp.targetEnemy = newTargetEnemy

    """ # Deprecated?
    # Autonomous Toggle Function
    def toggleAuto(self, blimpIDs):
        # If BlimpIDs is not a list
        if (type(blimpIDs) != list):
            blimpIDs = (blimpIDs)
        # Go through All Blimps
        for blimpID in blimpIDs:
            blimp = self.findBlimp(blimpID)
            # If Autonomous is False, Change to True
            if (blimp.auto == 0):
                self.parameterMessages.append((blimpID, self.pCodes["autoOn"]))
                blimp.auto = 1
            # If Autonomous is True, Change to False
            else:
                self.parameterMessages.append((blimpID, self.pCodes["autoOff"]))
                blimp.auto = 0
    """

    """ # Deprecated?
    # ======================== Update Grabber ========================= #
    # Description #
    # Updates Blimp IDs as a list
    def updateGrabber(self, blimpIDs):
        if (type(blimpIDs) != list):
            blimpIDs = (blimpIDs)
        for blimpID in blimpIDs:
            self.parameterMessages.append((blimpID,self.pCodes["toggleABG"]))
    """

    """ # Deprecated?
    # ========================== Find Blimp =========================== #
    # Description #
    # Returns Blimp Object
    def findBlimp(self, blimpID):
        for blimp in self.blimps:
            if (blimp.ID == blimpID):
                return blimp
    """

    # ================== Check if Input is a Integer ================== #
    # Description #
    # Returns true or false for the inputString (Integer == True)
    def isInt(self, inputString):
        try:
            int(inputString)
            return True
        except ValueError:
            return False

    # =================== Check if Input is a Float =================== #
    # Description #
    # Returns true or false for the inputString (Float == True)
    def isFloat(self, inputString):
        try:
            float(inputString)
            return True
        except ValueError:
            return False

    """ # Deprecated?
    # ======================== Get Blimp Name ========================= #
    # Description #
    # Returns the name of a Blimp based on it's ID (No Name = Blimp ID)
    def getBlimpName(self, ID):
        name = self.blimpIDNameMap.get(ID)
        if (name is None):
            name = "Blimp " + str(ID)
        return name
    """

    # ======================= Add a Fake Blimp ======================== #
    # Description #
    # Creates and Returns a Temporary Blimp with an ID and Name
    def addFakeBlimp(self, ID, name):
        fakeBlimp = Blimp(ID, name)
        fakeBlimp.lastHeartbeatDetected = 99999999999999
        self.swampBlimps[ID] = fakeBlimp

    # ======================= Get Blimp Index ======================== #
    # Description #
    # Get blimp's index in list from name
    def getBlimpIndex(self, blimpName, recursiveLevel=0):
        if recursiveLevel == 2:
            return -1
        if blimpName in self.blimpIndexMap:
            possibleIndex = self.blimpIndexMap[blimpName]
            possibleBlimp = self.blimps[possibleIndex]
            if blimpName == possibleBlimp.name:
                return possibleIndex
        self.fixBlimpIndexMap()
        return self.getBlimpIndex(blimpName,recursiveLevel+1)

    # ======================= Fix Blimp Index Map ======================== #
    # Description #
    # Empties the blimp index map and fills it with the current indices
    def fixBlimpIndexMap(self):
        self.blimpIndexMap = {}
        for i in range(0,len(self.blimps)):
            self.blimpIndexMap[self.blimps[i].name] = i

    def getOrderedConnectedBlimpIDs(self):
        connectedIDs = []
        for ID in self.swampBlimps.keys():
            blimp = self.swampBlimps[ID]
            if blimp.connected:
                connectedIDs.append(ID)
        connectedIDs.sort()
        return connectedIDs
