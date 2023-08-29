from V2_InProgress.Inputs.Input import Input
import time


class ControllerInput(Input):
    # ============================== INIT ==============================
    def __init__(self, name, data):
        self.controller = data
        super().__init__(name)

        self.controllerActionMapping = {"grab": "BUMPER_RIGHT",
                                        "auto": "TRIGGER_RIGHT",
                                        "shoot": "BUMPER_LEFT",
                                        "panicAuto": "Y",
                                        "kill": "X",
                                        "record": "A",
                                        "connectDown": "DPAD_DOWN",
                                        "connectUp": "DPAD_UP",
                                        "vibeRight": "DPAD_RIGHT",
                                        "vibeLeft": "DPAD_LEFT"}

        self.controllerInputMapping = {"Xbox 360 Wireless Receiver":  # in use
                                           {"JS_LEFT_X": "A0",
                                            "JS_LEFT_Y": "A1",
                                            "TRIGGER_LEFT": "A2",
                                            "JS_RIGHT_X": "A3",
                                            "JS_RIGHT_Y": "A4",
                                            "TRIGGER_RIGHT": "A5",
                                            "A": "B0",
                                            "B": "B1",
                                            "X": "B2",
                                            "Y": "B3",
                                            "BUMPER_LEFT": "B4",
                                            "BUMPER_RIGHT": "B5",
                                            "BACK": "B6",
                                            "START": "B7",
                                            "HOME": "B8",
                                            "JS_LEFT_BUTTON": "B9",
                                            "JS_RIGHT_BUTTON": "B10",
                                            "DPAD_LEFT": "B11",
                                            "DPAD_RIGHT": "B12",
                                            "DPAD_UP": "B13",
                                            "DPAD_DOWN": "B14"},
                                       "Xbox Series X Controller":
                                           {"JS_LEFT_X": "A0",
                                            "JS_LEFT_Y": "A1",
                                            "TRIGGER_LEFT": "A2",
                                            "JS_RIGHT_X": "A3",
                                            "JS_RIGHT_Y": "A4",
                                            "TRIGGER_RIGHT": "A5",
                                            "A": "B0",
                                            "B": "B1",
                                            "X": "B2",
                                            "Y": "B3",
                                            "BUMPER_LEFT": "B4",
                                            "BUMPER_RIGHT": "B5",
                                            "BACK": "B10",
                                            "START": "B11",
                                            "JS_LEFT_BUTTON": "B13",
                                            "JS_RIGHT_BUTTON": "B14",
                                            "DPAD_LEFT": "H00-",
                                            "DPAD_RIGHT": "H00+",
                                            "DPAD_UP": "H01+",
                                            "DPAD_DOWN": "H01-"},
                                       "Xbox 360 Controller":
                                           {"JS_LEFT_X": "A0",
                                            "JS_LEFT_Y": "A1",
                                            "TRIGGER_LEFT": "A2",
                                            "JS_RIGHT_X": "A3",
                                            "JS_RIGHT_Y": "A4",
                                            "TRIGGER_RIGHT": "A5",
                                            "A": "B0",
                                            "B": "B1",
                                            "X": "B2",
                                            "Y": "B3",
                                            "BUMPER_LEFT": "B4",
                                            "BUMPER_RIGHT": "B5",
                                            "BACK": "B6",
                                            "START": "B7",
                                            "JS_LEFT_BUTTON": "B9",
                                            "JS_RIGHT_BUTTON": "B10",
                                            "DPAD_LEFT": "H00-",
                                            "DPAD_RIGHT": "H00+",
                                            "DPAD_UP": "H01+",
                                            "DPAD_DOWN": "H01-"}
                                           }

        for key in self.controllerActionMapping.keys():
            self.actionStates[key] = False
            self.actionStartTime[key] = -1
            self.actionObserved[key] = False

        self.vibrateUntilTime = 0

    # ============================== ABSTRACT UPDATES ==============================
    def _Input__updateInputs(self):
        # Input
        controller = self.controller
        leftX = self.getControllerInput("JS_LEFT_X")
        leftY = -1 * self.getControllerInput("JS_LEFT_Y")
        # rightX = controller.get_axis(2) 2=left js; 5=right trigger
        rightX = self.getControllerInput("JS_RIGHT_X")
        rightY = -1 * self.getControllerInput("JS_RIGHT_Y")

        # Enforce dead-zones
        leftX = deadzones(leftX)
        rightX = deadzones(rightX)
        leftY = deadzones(leftY)
        rightY = deadzones(rightY)

        # Vibration
        if self.vibrateUntilTime > time.time():
            controller.rumble(1, 1, 0)
        else:
            controller.stop_rumble()

        mode = 1
        if mode == 1:
            self.recordedInput = [leftX, leftY, rightX, rightY]
        elif mode == 2:
            self.recordedInput = [leftX, rightY, rightX, leftY]

    def _Input__updateActions(self):
        currentTime = time.time()
        for actionName in self.controllerActionMapping.keys():
            if self.controllerActionMapping[actionName] is None:
                continue
            inputState = 1 if self.getControllerInput(self.controllerActionMapping[actionName]) > 0.5 else 0
            startTime = self.actionStartTime[actionName]
            elapsedTimeRequired = self.actionElapsedTimeRequired[actionName]
            enoughTime = currentTime - startTime >= elapsedTimeRequired
            observed = self.actionObserved[actionName]
            if not inputState and startTime == -1:
                pass
            elif not inputState and startTime != -1:
                self.actionStates[actionName] = False
                self.actionStartTime[actionName] = -1
            elif inputState and startTime != -1 and not enoughTime:
                pass
            elif inputState and startTime != -1 and enoughTime and not observed:
                self.actionStates[actionName] = True
                self.actionObserved[actionName] = True
            elif inputState and startTime != -1 and enoughTime and observed:
                self.actionStates[actionName] = False
            elif inputState and startTime == -1:
                self.actionStartTime[actionName] = currentTime
                self.actionObserved[actionName] = False

    # Used to set the time until which the controller will vibrate
    def notify(self, timeDuration):
        # print("Notify")
        self.vibrateUntilTime = time.time() + timeDuration

    # ============================== HELPER FUNCTIONS ==============================
    # Gets requested input state from controller
    # Takes in requested input name -> uses controller model to find input code -> gets input from controller
    def getControllerInput(self, inputName):
        controllerName = self.controller.get_name()  # Returns model name of controller
        inputSource = self.controllerInputMapping[controllerName][inputName]  # Get input code for requested input
        # Axes
        if inputSource[0] == "A":
            axisNum = int(inputSource[1:])
            return self.controller.get_axis(axisNum)
        # Buttons
        elif inputSource[0] == "B":
            buttonNum = int(inputSource[1:])
            return self.controller.get_button(buttonNum)
        # Hats
        elif inputSource[0] == "H":
            hatNum = int(inputSource[1])
            hatIndex = int(inputSource[2])
            retCondition = inputSource[3]
            inputValue = self.controller.get_hat(hatNum)[hatIndex]
            if retCondition == "+" and inputValue == 1:
                return 1
            elif retCondition == "-" and inputValue == -1:
                return 1
        return 0

    # Useful function to help identify which inputs on a controller map to a given axis/button/hat
    # Prints out current states of all possible axes/buttons/hats
    def dumpControllerData(self):
        # Axes
        for i in range(20):
            try:
                print("Axis ", i, ": ", self.controller.get_axis(i), sep='')
            finally:
                break
        # Buttons
        for i in range(20):
            try:
                print("Button ", i, ": ", self.controller.get_button(i), sep='')
            finally:
                break
        # Hats
        for i in range(20):
            try:
                print("Hat ", i, ": ", self.controller.get_hat(i), sep='')
            finally:
                break
        print()
        # time.sleep(0.5)

    # Alternate function to list name and number of axes/trackballs/buttons/hats on controller
    # Does not print out input states, just the number of inputs
    def printControllerData(self):
        print("Input")
        print("Name:", self.controller.get_name())
        print("Axes:", self.controller.get_numaxes())
        print("Trackballs:", self.controller.get_numballs())
        print("Buttons:", self.controller.get_numbuttons())
        print("Hats:", self.controller.get_numhats())

# Enforce dead-zones around -1, 0, and 1
def deadzones(x, deadZero=0.1, deadOne=0.01, decimals=2):
    if (abs(x) < deadZero): return 0
    if (x > 1 - deadOne): return 1
    if (x < -1 + deadOne): return -1
    return round(x, decimals)


