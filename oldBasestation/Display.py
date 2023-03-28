# Imported Classes
import sys
import pygame
import math
import os
from pygame.locals import *
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
from Text import getTextSurface

class Display:
    def __init__(self,blimpHandler):
        self.activeController = 0
        #Store BlimpHandler parameter
        self.blimpHandler = blimpHandler
        blimpHandler.setDisplay(self)
        self.alive = True

        # Anchors
        self.width_screen = 1200
        self.height_screen = 600
        self.anchor_x_inputVisual = 0
        self.anchor_y_inputVisual = 0
        self.width_inputVisual = 200
        self.anchor_x_inputVisualRect = self.anchor_x_inputVisual + 20
        self.anchor_y_inputVisualRect = self.anchor_y_inputVisual + 35
        self.width_inputVisualRect = 160
        self.height_inputVisualRect = 80
        self.inputVisualRect_radius = 35
        self.inputVisualRect_JSCircle = 5
        self.anchor_x_input = self.anchor_x_inputVisual + self.width_inputVisual
        self.anchor_y_input = self.anchor_y_inputVisual
        self.width_input = 225
        self.align_input_right = self.anchor_x_input+120
        self.anchor_y_inputText = 60
        self.spacing_y_inputText = 100
        self.anchor_x_blimps = self.anchor_x_input + self.width_input
        self.anchor_y_blimps = 0
        self.align_blimps_left = self.anchor_x_blimps
        self.width_blimps = 250
        self.anchor_y_blimpText = 60
        self.spacing_y_blimpText = 45
        self.anchor_x_MPB = self.anchor_x_blimps + 125
        self.anchor_y_MPB = 66
        self.spacing_y_MPB = self.spacing_y_inputText
        self.anchor_x_activeController = self.anchor_x_blimps + self.width_blimps
        self.anchor_y_activeController = 0
        self.anchor_x_activeControllerIndicator = self.anchor_x_input + 10
        self.anchor_y_activeControllerIndicator = 73
        self.anchor_x_blimpState = self.anchor_x_blimps + self.width_blimps
        self.anchor_y_blimpState = 15
        self.width_blimpState = 200

        self.anchor_x_targetGoal = self.anchor_x_blimpState + self.width_blimpState
        self.anchor_y_targetGoal = self.anchor_y_blimpState
        self.width_targetGoal = 180
        self.anchor_x_targetEnemy = self.anchor_x_targetGoal + self.width_targetGoal
        self.anchor_y_targetEnemy = self.anchor_y_targetGoal
        self.width_targetEnemy = 180

        self.anchor_x_blimpStateLegend = self.width_screen - 200
        self.anchor_y_blimpStateLegend = self.height_screen - 100
        self.spacing_y_blimpStateLegend = 50

        self.anchor_x_barometer = 50
        self.anchor_y_barometer = self.height_screen - 50

        self.anchor_x_plotNames = 25
        self.anchor_y_plotNames = 25
        self.spacing_y_plotNames = 100
        self.anchor_x_plot = 150
        self.anchor_y_plot = 50
        self.width_plot = self.width_screen - self.anchor_x_plot - 50
        self.height_plot = self.height_screen - self.anchor_y_plot - 50

        #Colors
        self.activeColor = Color(0,255,255)
        self.color_inputVisual_background = Color(100,100,100)
        self.color_inputVisual_grid = Color(150,150,150)
        self.color_inputVisual_joystick = Color(255,255,255)
        self.color_blimpState_autonomous = Color(0,255,0)
        self.color_blimpState_manual = Color(0,0,0)
        self.color_plot_screenbackground = Color(150,150,150)
        self.color_plot_graphbackground = Color(200,200,200)
        self.color_plot_data = Color(0,0,0)
        self.color_goal_yellow = Color(209,245,66)
        self.color_goal_orange = Color(245,129,66)
        self.color_enemy_red = Color(255,0,0)
        self.color_enemy_green = Color(0,255,0)
        self.color_enemy_blue = Color(0,0,255)

        # Game Display
        print("Beginning Program.")
        self.screen = pygame.display.set_mode((self.width_screen, self.height_screen))
        print("Size:",self.screen.get_size())
        pygame.display.set_caption("Multi-Blimp LTA Control")

        # Set up screen
        self.background = pygame.Surface(self.screen.get_size())
        self.background = self.background.convert()
        self.background.fill((200, 200, 200))
        self.screen.blit(self.background, (0, 0))

        #Set up variables
        self.textSurfaces = {};

        self.textSurface_Input = getTextSurface("Input:",50)
        self.textSurface_Blimps = getTextSurface("Blimps:",50)
        self.textSurface_State = getTextSurface("State",30)
        self.textSurface_Status = getTextSurface("Status",30)
        #self.textSurface_Clamp = getTextSurface("Clamp",30)
        self.textSurface_Record = getTextSurface("\"r\" to record",30)
        self.textSurface_Panic = getTextSurface("\"p\" for autonomous panic",30)

        #Multi-Purpose Button (MPB)
        self.MPBSurface_Enabled = pygame.Surface((15,15))
        self.MPBSurface_Enabled = self.MPBSurface_Enabled.convert()
        self.MPBSurface_Enabled.fill((200,0,200))
        self.MPBSurface_Disabled = pygame.Surface((15,15))
        self.MPBSurface_Disabled = self.MPBSurface_Disabled.convert()
        self.MPBSurface_Disabled.fill((0,255,0))
        self.buttons = []
        #self.buttons.append(((0,0),self.textSurface_Input.get_size(),"InputLabel"))
        self.drawing = False

        #Create state surfaces
        stateStringMap = blimpHandler.blimpStateStrings
        self.stateSurfaceMap = {}
        for key in stateStringMap.keys():
            self.stateSurfaceMap[key] = getTextSurface(stateStringMap[key],30)

        #Render mode
        self.renderMode = "InputsBlimps" #InputsBlimps, Plots

        print("Display Initialized")

    def handleEvent(self,event):
        if event.type == QUIT:
            print("Attempted Quit")
            self.alive = False
        if event.type == KEYDOWN:
            if self.getKey(K_ESCAPE):
                print("Escape key pressed; Aborting.")
                self.alive = False
            elif self.getKey(K_p):
                for blimp in self.blimpHandler.blimps:
                    blimp.auto = 1
                print("Hardcoded autonomous panic!")

        if event.type == MOUSEBUTTONDOWN:
            if(event.button == 1): #Left Click
                pos = pygame.mouse.get_pos()
                for button in self.buttons:
                    if(self.posInRange(pos,button[0],button[1])):
                        self.handleButton(button[2])
                inputs = self.blimpHandler.inputHandler.inputs
                for i in range(0, len(inputs)):
                    if(self.inRangeInput(pos,i)):
                        self.drawing = True
                        self.drawingIndex = i
        if event.type == MOUSEBUTTONUP:
            if(event.button == 1): #Left Click
                pos = pygame.mouse.get_pos()
                inputs = self.blimpHandler.inputHandler.inputs
                blimps = self.blimpHandler.blimps

                for i in range(0,len(inputs)):
                    if(self.inRangeInput(pos,i)):
                        if(self.activeController == i):
                            self.activeController = -1
                        else:
                            self.activeController = i

                for i in range(0,len(blimps)):
                    if(self.inRangeBlimp(pos,i)):
                        if(self.drawing):
                            inputName = self.blimpHandler.inputHandler.inputs[self.drawingIndex].name
                            blimpName = self.blimpHandler.blimps[i].name
                            self.blimpHandler.blimpMapper.updateMapping(inputName, blimpName)
                self.drawing = False

    def handleButton(self,name):
        #print("handle button")
        if(name=="InputLabel"):
            print("Did not reinitialize inputs")
            #self.blimpHandler.initInputs()
        elif(name[0:3]=="MPB"):
            print("MPB pressed")
            blimpID = name[3:]
            self.blimpHandler.pushMPB(blimpID)
        elif(name[0:2]=="TG"):
            print("TG button pressed")
            blimpID = name[2:]
            self.blimpHandler.pushTGButton(blimpID)
        elif(name[0:2]=="TE"):
            print("TE button pressed")
            blimpID = name[2:]
            self.blimpHandler.pushTEButton(blimpID)

    def posInRange(self,pos,origin,size):
        xrange = (origin[0], origin[0] + size[0])
        yrange = (origin[1], origin[1] + size[1])
        validX = xrange[0] <= pos[0] and pos[0] <= xrange[1]
        validY = yrange[0] <= pos[1] and pos[1] <= yrange[1]
        return validX and validY

    def inRangeInput(self, pos, index):
        size = self.blimpHandler.inputHandler.inputs[index].getNameSurface().get_size()
        inputTextY = self.anchor_y_inputText + index*self.spacing_y_inputText
        return self.posInRange(pos,(self.align_input_right - size[0],inputTextY),size)

    def inRangeBlimp(self, pos, index):
        size = self.blimpHandler.blimps[index].getNameSurface().get_size()
        blimpTextY = self.anchor_y_blimpText + index*self.spacing_y_blimpText
        return self.posInRange(pos,(self.align_blimps_left,blimpTextY),size)

    """
    def update(self):
        for event in pygame.event.get():
            self.handleEvent(event)
        self.draw()
    """

    def updateEvent(self):
        for event in pygame.event.get():
            self.handleEvent(event)

    def updateDraw(self):
        self.draw()

    def draw(self):
        if(self.renderMode == "InputsBlimps"):
            self.draw_InputsBlimps()
        elif(self.renderMode == "Plots"):
            self.draw_Plots()
        pygame.display.update()

    def draw_InputsBlimps(self):
        self.drawMappings()
        #self.drawActiveController()
        self.drawMisc()

    def drawMappings(self):
        inputs = self.blimpHandler.inputHandler.inputs
        blimpIDs = self.blimpHandler.getOrderedConnectedBlimpIDs()

        #backWidth = self.width_inputVisual + self.width_input + self.width_blimps + self.width_blimpState + self.width_blimpStatus
        backWidth = self.width_screen
        pygame.draw.rect(self.screen,Color(150,150,150),Rect(self.anchor_x_inputVisual,self.anchor_y_inputVisual,backWidth,self.height_screen)) #Draw background
        self.screen.blit(self.getTextSurface("Input:",50),(self.anchor_x_input,self.anchor_y_input))
        self.screen.blit(self.getTextSurface("Blimps:",50),(self.anchor_x_blimps,self.anchor_y_blimps))
        self.screen.blit(self.getTextSurface("State",30),(self.anchor_x_blimpState,self.anchor_y_blimpState))
        self.screen.blit(self.getTextSurface("TargetGoal",20),(self.anchor_x_targetGoal,self.anchor_y_targetGoal))
        self.screen.blit(self.getTextSurface("TargetEnemy",20),(self.anchor_x_targetEnemy,self.anchor_y_targetEnemy))
        #Iterate through inputs
        for i in range(0,len(inputs)):
            #Render inputs
            inputSurface = inputs[i].getNameSurface()
            inputTextX = self.align_input_right-inputSurface.get_size()[0]
            inputTextY = self.anchor_y_inputText + i*self.spacing_y_inputText
            self.screen.blit(inputSurface,(inputTextX,inputTextY))
            #Render input visuals
            rectX = self.anchor_x_inputVisualRect
            rectY = self.anchor_y_inputVisualRect + i*self.spacing_y_inputText
            pygame.draw.rect(self.screen, self.color_inputVisual_background, Rect(rectX, rectY, self.width_inputVisualRect, self.height_inputVisualRect))
            pygame.draw.line(self.screen, self.color_inputVisual_grid, (rectX, rectY + 0.5*self.height_inputVisualRect), (rectX+self.width_inputVisualRect, rectY + 0.5*self.height_inputVisualRect))
            pygame.draw.line(self.screen, self.color_inputVisual_grid, (rectX+0.25*self.width_inputVisualRect, rectY), (rectX+0.25*self.width_inputVisualRect, rectY+self.height_inputVisualRect))
            pygame.draw.line(self.screen, self.color_inputVisual_grid, (rectX+0.75*self.width_inputVisualRect, rectY), (rectX+0.75*self.width_inputVisualRect, rectY+self.height_inputVisualRect))
            input = inputs[i].grabInput()
            leftJS = (rectX+0.25*self.width_inputVisualRect + input[0] * self.inputVisualRect_radius, rectY + 0.5*self.height_inputVisualRect - input[1] * self.inputVisualRect_radius)
            rightJS = (rectX+0.75*self.width_inputVisualRect + input[2] * self.inputVisualRect_radius, rectY + 0.5*self.height_inputVisualRect - input[3] * self.inputVisualRect_radius)
            pygame.draw.circle(self.screen, self.color_inputVisual_joystick, leftJS, self.inputVisualRect_JSCircle)
            pygame.draw.circle(self.screen, self.color_inputVisual_joystick, rightJS, self.inputVisualRect_JSCircle)


        #Render ActiveController Indicator
        if(self.activeController != -1):
            indicatorX = self.anchor_x_activeControllerIndicator
            indicatorY = self.anchor_y_activeControllerIndicator + self.activeController * self.spacing_y_inputText
            pygame.draw.circle(self.screen, self.activeColor, (indicatorX, indicatorY), 5)

        #Render list of blimps
        for i in range(0,len(blimpIDs)):
            blimpID = blimpIDs[i]
            blimp = self.blimpHandler.swampBlimps[blimpID]
            if blimp.auto == 1:
                blimpColor = self.color_blimpState_autonomous
            else:
                blimpColor = self.color_blimpState_manual
            blimpSurface = self.getTextSurface(blimp.name, int(40 - len(blimp.name)),blimpColor)
            blimpSurface = blimpSurface.convert_alpha()
            #Render blimp heartbeats
            heartbeatWidth = blimpSurface.get_width()*blimp.lastHeartbeatDiff/blimp.heartbeatDisconnectDelay
            heartbeatRect = Rect(0,0,heartbeatWidth,blimpSurface.get_height())
            blimpSurface.fill(Color(255,255,255,50),heartbeatRect,special_flags=BLEND_RGBA_ADD)
            blimpTextX = self.align_blimps_left
            blimpTextY = self.anchor_y_blimpText + i*self.spacing_y_blimpText
            # Render blimp name
            self.screen.blit(blimpSurface,(blimpTextX,blimpTextY))
            # Render blimp state
            self.screen.blit(self.stateSurfaceMap[blimp.receivedState],(self.anchor_x_blimpState,blimpTextY))
            # Render blimp targetGoalColor
            targetGoalColor = self.color_plot_screenbackground # Assume background as default color
            if blimp.targetGoal == "Y":
                targetGoalColor = self.color_goal_yellow
            elif blimp.targetGoal == "O":
                targetGoalColor = self.color_goal_orange
            pygame.draw.rect(self.screen, targetGoalColor, Rect(self.anchor_x_targetGoal+35,blimpTextY,25,25))
            buttonLabel = "TG" + str(blimp.ID) # TG = TargetGoal
            if not self.buttonLabelExists(buttonLabel):
                newButton = ((self.anchor_x_targetGoal+35, blimpTextY), (25, 25), buttonLabel)
                self.buttons.append(newButton)
            # Render blimp targetEnemyColor
            targetEnemyColor = self.color_plot_screenbackground # Assume background color as default
            if blimp.targetEnemy == "R":
                targetEnemyColor = self.color_enemy_red
            elif blimp.targetEnemy == "G":
                targetEnemyColor = self.color_enemy_green
            elif blimp.targetEnemy == "B":
                targetEnemyColor = self.color_enemy_blue
            pygame.draw.rect(self.screen,targetEnemyColor,Rect(self.anchor_x_targetEnemy+45,blimpTextY,25,25))
            buttonLabel = "TE" + str(blimp.ID) # TE = TargetEnemy
            if not self.buttonLabelExists(buttonLabel):
                newButton = ((self.anchor_x_targetEnemy+45, blimpTextY), (25, 25), buttonLabel)
                self.buttons.append(newButton)

        # Render mapping line if mouse is to the right of inputs
        newLineColor = Color(255, 100, 255)
        lineColor = Color(255, 255, 255)
        lineThickness = 3
        pos = pygame.mouse.get_pos()
        if self.drawing and pos[0] > self.align_input_right:
            inputSurfaceSize = inputs[self.drawingIndex].getNameSurface().get_size()
            inputPos = (self.align_input_right,int(self.anchor_y_inputText + self.drawingIndex*self.spacing_y_inputText+inputSurfaceSize[1]/2))
            pygame.draw.line(self.screen,newLineColor,inputPos,pos,lineThickness)

        # Render mapping lines
        blimpIndexMap = {}
        for i in range(0,len(blimpIDs)):
            blimpID = blimpIDs[i]
            blimpIndexMap[blimpID] = i
        inputIndexMap = {}
        for i in range(0,len(inputs)):
            inputName = inputs[i].name
            inputIndexMap[inputName] = i
        inputToBlimpMap = self.blimpHandler.blimpMapper.inputToBlimpMap
        for inputName in inputToBlimpMap.keys():
            blimpID = inputToBlimpMap[inputName]
            blimp = self.blimpHandler.swampBlimps[blimpID]
            inputIndex = inputIndexMap[inputName]
            blimpIndex = blimpIndexMap[blimpID]
            try:
                inputSurfaceSize = inputs[inputIndex].getNameSurface().get_size()
                blimpSurfaceSize = blimp.getNameSurface().get_size()
                inputPos = (self.align_input_right,int(self.anchor_y_inputText + inputIndex*self.spacing_y_inputText+inputSurfaceSize[1]/2))
                blimpPos = (self.align_blimps_left,int(self.anchor_y_blimpText + blimpIndex*self.spacing_y_blimpText+blimpSurfaceSize[1]/2))
                pygame.draw.line(self.screen, lineColor, inputPos, blimpPos, lineThickness)
            except IndexError:
                print("Input Disagreement")

    def drawActiveController(self):
        activeColor = Color(0,255,255)
        whiteColor = Color(255,255,255)

        #Joystick variables
        JSLeftOrigin = (600+200,200)
        JSRightOrigin = (1000+200,200)
        circleRadius = 25
        joystickRadius = 200-circleRadius

        #Clamp variables
        CLeftOrigin = (490+200,520)
        CRightOrigin = (510+200,520)
        CLength = 70

        #Draw background with active outline
        pygame.draw.rect(self.screen,activeColor,Rect(400+200,0,800,600))
        pygame.draw.rect(self.screen,Color(100,100,100),Rect(405+200,5,790,390))
        pygame.draw.rect(self.screen,Color(100,100,100),Rect(405+200,405,790,190))

        #Display Clamp text
        self.screen.blit(self.textSurface_Clamp, (465+200,542))

        if(self.activeController==-1):
            #Joysticks
            pygame.draw.circle(self.screen,whiteColor,JSLeftOrigin,circleRadius)
            pygame.draw.circle(self.screen,whiteColor,JSRightOrigin,circleRadius)
            #Clamp
            pygame.draw.line(self.screen,whiteColor,CLeftOrigin,self.getClampPoint(CLeftOrigin, CLength,180),5)
            pygame.draw.line(self.screen,whiteColor,CRightOrigin,self.getClampPoint(CRightOrigin, CLength,0),5)
        else:
            #Joysticks
            input = self.blimpHandler.inputHandler.inputs[self.activeController].getInput()
            leftPos = (JSLeftOrigin[0]+joystickRadius*input[0],JSLeftOrigin[1]-joystickRadius*input[1])
            rightPos = (JSRightOrigin[0]+joystickRadius*input[2],JSRightOrigin[1]-joystickRadius*input[3])
            pygame.draw.circle(self.screen,whiteColor,leftPos,circleRadius)
            pygame.draw.circle(self.screen,whiteColor,rightPos,circleRadius)
            #Clamp
            clampValue = 0
            pygame.draw.line(self.screen, whiteColor, CLeftOrigin, self.getClampPoint(CLeftOrigin, CLength, 180-clampValue*90), 5)
            pygame.draw.line(self.screen, whiteColor, CRightOrigin, self.getClampPoint(CRightOrigin, CLength, clampValue*90), 5)

    def drawMisc(self):
        #self.screen.blit(self.textSurface_Record, (self.anchor_x_keybind,self.anchor_y_keybind))
        #self.screen.blit(self.textSurface_Panic, (self.anchor_x_keybind,self.anchor_y_keybind+self.spacing_y_keybind))
        self.screen.blit(self.getTextSurface("Manual",30,self.color_blimpState_manual),(self.anchor_x_blimpStateLegend,self.anchor_y_blimpStateLegend))
        self.screen.blit(self.getTextSurface("Autonomous",30,self.color_blimpState_autonomous),(self.anchor_x_blimpStateLegend,self.anchor_y_blimpStateLegend+self.spacing_y_blimpStateLegend))
        stringBarometer = "Barometer: "
        if(self.blimpHandler.baroType == None):
            stringBarometer += "Disconnected"
        else:
            stringBarometer += "(" + self.blimpHandler.baroType + ") " + str(self.blimpHandler.baseHeight)
        self.screen.blit(self.getTextSurface(stringBarometer, 30),(self.anchor_x_barometer, self.anchor_y_barometer))

    def draw_Plots(self):
        pygame.draw.rect(self.screen,self.color_plot_screenbackground,Rect(0,0,self.width_screen,self.height_screen)) #Draw background

        plotData = self.blimpHandler.plotData
        plotKeys = list(plotData.keys())

        #Render plot variables
        for keyIndex in range(0,len(plotKeys)):
            key = plotKeys[keyIndex]
            self.screen.blit(self.getTextSurface(key, 30),(self.anchor_x_plotNames, self.anchor_y_plotNames + keyIndex * self.spacing_y_plotNames))

        #Render plot background
        pygame.draw.rect(self.screen, self.color_plot_graphbackground,Rect(self.anchor_x_plot,self.anchor_y_plot,self.width_plot,self.height_plot))

        #Render active plot
        if(len(plotKeys) > 0):
            varPlotData = plotData.get(plotKeys[0])
            data = varPlotData[0]
            dataMin = varPlotData[1]
            dataMax = varPlotData[2]

            if(len(data) > 1):
                xMin = data[0][0]
                xMax = data[len(data)-1][0]
                for dataPoint in data:
                    dataPointCenter = (self.anchor_x_plot + (dataPoint[0]-xMin)/(xMax-xMin)*self.width_plot, self.anchor_y_plot + self.height_plot/2 - (dataPoint[1]-dataMin)/(dataMax-dataMin)*self.height_plot)
                    pygame.draw.circle(self.screen,self.color_plot_data,dataPointCenter,2)

    def getTextSurface(self, text, size, color=None):
        if(color == None):
            color = Color(0,0,0)
        textKey = text + "," + str(size) + "," + str(color)
        surface = self.textSurfaces.get(textKey)
        if(surface == None):
            surface = getTextSurface(text,size,color)
            self.textSurfaces[textKey] = surface
            #print("New key:",textKey)
            #print(len(self.textSurfaces.keys()))
        return surface

    def getClampPoint(self, startingPoint, length, angleDegrees):
        angle = angleDegrees/180*math.pi
        return (startingPoint[0]+length*math.cos(angle), startingPoint[1]-length*math.sin(angle))

    def removeBlimp(self, blimpID):
        for i in range(len(self.buttons)-1,-1,-1):
            if(self.buttons[i][2]=="MPB"+str(blimpID)):
                self.buttons.pop(i)
            elif(self.buttons[i][2]=="TG"+str(blimpID)):
                self.buttons.pop(i)
            elif(self.buttons[i][2]=="TE"+str(blimpID)):
                self.buttons.pop(i)

    def getElementY(self, index):
        return 60+60*index

    def getKey(self, key):
        return pygame.key.get_pressed()[key]

    def getNumber(self):
        numIDs = [K_1, K_2, K_3, K_4, K_5, K_6, K_7, K_8, K_9, K_0]
        for i in range(0,len(numIDs)):
            if(self.getKey(numIDs[i])): return (True,i+1)
        return (False,-1)

    def close(self):
        print("Closing Display")
        pygame.quit()
        sys.exit()

    def buttonLabelExists(self, label):
        for button in self.buttons:
            if(button[2] == label):
                return True
        return False

