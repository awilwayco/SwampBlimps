from Text import getTextSurface

class Blimp:
    def __init__(self,ID,name):
        self.IP = 1
        self.ID = ID
        self.auto = 0
        self.grabbing = 0
        self.shooting = 0
        self.connected = False
        self.targetGoal = "Y" #"Y","O"
        self.targetEnemy = "B" #"R","G","B"

        self.receivedAuto = "Null"
        self.receivedStatus = "Null"

        self.receivedAuto_Surface = getTextSurface(self.receivedAuto,25)
        self.receivedStatus_Surface = getTextSurface(self.receivedStatus,25)

        self.surfaces = {}

        self.name = name
        self.nameSurface = getTextSurface(self.name, int(40 - len(self.name)))
        self.lastHeartbeatDetected = 0
        self.lastHeartbeatDiff = 0
        self.heartbeatDisconnectDelay = 5 #seconds
        self.lastTimeInputDataSent = 0
        self.timeInputDelay = 0.05 #seconds
        self.lastBarometerSentTime = 0
        self.barometerSendDelay = 1/20 #seconds
        self.lastTargetGoalSentTime = 0
        self.targetGoalSendDelay = 1 #seconds

        self.data = []
        self.receivedState = -1

    def getNameSurface(self):
        return self.nameSurface

    def toggleIMU(self):
        self.IMUEnabled = 1 - self.IMUEnabled