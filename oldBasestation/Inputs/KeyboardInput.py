from V2_InProgress.Inputs.Input import Input
import time
import pygame
from pygame.locals import *


class KeyboardInput(Input):
    # ============================== INIT ==============================
    def __init__(self, name, data):
        self.keys = data
        super().__init__(name)

        self.keyboardActionMapping = {"grab": K_g,
                                      "auto": K_a,
                                      "shoot": K_l,
                                      "panicAuto": K_p,
                                      "kill": K_k,
                                      "record": K_r,
                                      "connectDown": None,
                                      "connectUp": None,
                                      "vibeRight": None,
                                      "vibeLeft": None}

        for key in self.keyboardActionMapping.keys():
            self.actionStates[key] = False
            self.actionStartTime[key] = -1
            self.actionObserved[key] = False

    # ============================== ABSTRACT UPDATES ==============================
    def _Input__updateInputs(self):
        # Input
        keys = self.keys  # KeyConstants=[right,left,forward,backward,up,down,morePower,grab,auto]
        powerNormal = 0.3
        powerAdd = 0.2
        power = powerNormal + powerAdd * getKey(keys[6])

        leftX = getKey(keys[0]) - getKey(keys[1])
        leftY = getKey(keys[2]) - getKey(keys[3])
        rightX = 0  # self.getKey(keys[3]) - self.getKey(keys[4])
        rightY = getKey(keys[4]) - getKey(keys[5])

        leftX *= (power+0.3)
        leftY *= (power+0.3)
        rightX *= power
        rightY *= (power+0.5)

        self.recordedInput = [leftX, leftY, rightX, rightY]

    def _Input__updateActions(self):
        currentTime = time.time()
        for actionName in self.keyboardActionMapping.keys():
            if self.keyboardActionMapping[actionName] is None:
                continue
            inputState = getKey(self.keyboardActionMapping[actionName])
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

    def notify(self, timeDuration):
        # Keyboard can't vibrate
        pass


# ============================== STATIC HELPER FUNCTIONS ==============================
def getKey(key):
    return pygame.key.get_pressed()[key]
