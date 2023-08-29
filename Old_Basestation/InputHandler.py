from Inputs.KeyboardInput import KeyboardInput
from Inputs.ControllerInput import ControllerInput
import pygame
from pygame.locals import *


class InputHandler:
    def __init__(self):
        self.inputsStatic = []
        self.inputsJoysticks = []
        self.joystickCount = 0
        self.inputs = []
        self.inputIndexMap = {}

        self.updateInputs()

    def updateInputs(self):
        # Updates static/joystick input arrays, fills total input array with static/joystick inputs
        self.updateInputsStatic()
        self.updateInputsJoysticks()

        self.inputs = []
        self.inputs.extend(self.inputsStatic)
        self.inputs.extend(self.inputsJoysticks)

    # Clears static input array, manually fills it with static inputs (i.e. defined keyboard inputs)
    def updateInputsStatic(self):
        self.inputsStatic = []

        # Init WASD Input
        input_WASD = KeyboardInput("WASD", (K_d, K_a, K_w, K_s, K_UP, K_DOWN, K_c, K_e, K_q))
        self.inputsStatic.append(input_WASD)

    # Clears joystick input array, fills it with all currently connected joysticks
    def updateInputsJoysticks(self):
        self.inputsJoysticks = []
        self.joystickCount = pygame.joystick.get_count()
        for i in range(0, self.joystickCount):
            controller = pygame.joystick.Joystick(i)
            controller.init()
            # printControllerData(controller)
            controllerName = "Contrl " + str(controller.get_instance_id())
            input_Controller = ControllerInput(controllerName, controller)
            self.inputsJoysticks.append(input_Controller)

    def joystickCountMismatch(self):
        return pygame.joystick.get_count() != self.joystickCount

    def update(self):
        if self.joystickCountMismatch():
            self.updateInputs()
        for input in self.inputs:
            input.update()

    def fixInputIndexMap(self):
        self.inputIndexMap = {}
        for i in range(0,len(self.inputs)):
            self.inputIndexMap[self.inputs[i].name] = i

    def getInputByName(self, inputName):
        if inputName in self.inputIndexMap:
            possibleIndex = self.inputIndexMap[inputName]
            possibleInput = self.inputs[possibleIndex]
            if inputName == possibleInput.name:
                return possibleInput
        self.fixInputIndexMap()
        correctIndex = self.inputIndexMap[inputName]
        correctInput = self.inputs[correctIndex]
        return correctInput
