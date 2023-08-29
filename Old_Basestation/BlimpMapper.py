class BlimpMapper:
    # ============================== INIT ==============================
    def __init__(self, blimpHandler, inputHandler):
        self.blimpHandler = blimpHandler
        self.inputHandler = inputHandler

        self.inputToBlimpMap = {}
        self.blimpToInputMap = {}

        self.lastNumBlimps = 0
        self.lastNumInputs = 0

    def update(self):
        self.checkBadMappings()

    # Enforces valid mapping state
    def checkBadMappings(self):
        # Check for removed inputs
        if len(self.inputHandler.inputs) < self.lastNumInputs:
            # Uh oh, an input has been removed...
            # Make set of valid inputs
            validInputs = set()
            for input in self.inputHandler.inputs:
                validInputs.add(input.name)
            # Iterate through mappings, remove invalid inputs
            possibleInputs = list(self.inputToBlimpMap.keys())
            for possibleInput in possibleInputs:
                if not possibleInput in validInputs:
                    # Remove mapping!
                    invalidBlimp = self.inputToBlimpMap.pop(possibleInput)
                    self.blimpToInputMap.pop(invalidBlimp)
        self.lastNumInputs = len(self.inputHandler.inputs)
        # Check for removed blimps
        blimpIDs = self.blimpHandler.getOrderedConnectedBlimpIDs()
        if len(blimpIDs) < self.lastNumBlimps:
            # Uh oh, a blimp has been removed...
            # Make set of valid blimps
            validBlimps = set()
            for ID in blimpIDs:
                validBlimps.add(ID)
            # Iterate through mappings, remove invalid blimps
            possibleBlimps = list(self.blimpToInputMap.keys())
            for possibleBlimp in possibleBlimps:
                if not possibleBlimp in validBlimps:
                    # Remove mapping!
                    invalidInput = self.blimpToInputMap.pop(possibleBlimp)
                    self.inputToBlimpMap.pop(invalidInput)
        self.lastNumBlimps = len(blimpIDs)

    def updateMapping(self, inputName, blimpID):
        if inputName in self.inputToBlimpMap and blimpID in self.blimpToInputMap and self.inputToBlimpMap[inputName] == blimpID:
            # Mapping exists... remove it!
            self.inputToBlimpMap.pop(inputName)
            self.blimpToInputMap.pop(blimpID)
        else:
            # Mapping does not exist... create it!
            # Remove existing mappings between input
            if inputName in self.inputToBlimpMap:
                mappedBlimpID = self.inputToBlimpMap[inputName]
                self.inputToBlimpMap.pop(inputName)
                self.blimpToInputMap.pop(mappedBlimpID)
            if blimpID in self.blimpToInputMap:
                mappedInputName = self.blimpToInputMap[blimpID]
                self.blimpToInputMap.pop(blimpID)
                self.inputToBlimpMap.pop(mappedInputName)
            self.inputToBlimpMap[inputName] = blimpID
            self.blimpToInputMap[blimpID] = inputName

    # Assumes valid mapping state (see checkBadMapping)
    def getMappedBlimp(self, inputName):
        if inputName in self.inputToBlimpMap:
            blimpID = self.inputToBlimpMap[inputName]
            return self.blimpHandler.swampBlimps[blimpID]
        return None

    # Assumes valid mapping state (see checkBadMapping)
    def getMappedInput(self, blimpID):
        if blimpID in self.blimpToInputMap:
            inputName = self.blimpToInputMap[blimpID]
            return self.inputHandler.getInputByName(inputName)
        return None

    # Assumes valid mapping state (see checkBadMapping)
    def mapUp(self, inputName):
        blimpIDs = self.blimpHandler.getOrderedConnectedBlimpIDs()
        numBlimps = len(blimpIDs)
        # By default, start looking for new mappings from the end of the list, iterating upwards
        prevIndex = numBlimps
        # Check if there is an existing mapping to start from
        if inputName in self.inputToBlimpMap:
            # Mapping exists
            blimpID = self.inputToBlimpMap[inputName]
            self.updateMapping(inputName, blimpID)
            prevIndex = blimpIDs.index(blimpID)
        validMappingFound = False
        # Iterate through blimps to find an unmapped blimp to map to
        while prevIndex > 0:
            nextIndex = prevIndex - 1
            nextBlimpID = blimpIDs[nextIndex]
            if nextBlimpID in self.blimpToInputMap:
                # Mapping exists... move on
                prevIndex = nextIndex
            else:
                # Mapping doesn't exist... create it!
                self.updateMapping(inputName, nextBlimpID)
                validMappingFound = True
                break
        if not validMappingFound:
            # print("No valid blimps to map to.")
            pass

    def mapDown(self, inputName):
        blimpIDs = self.blimpHandler.getOrderedConnectedBlimpIDs()
        numBlimps = len(blimpIDs)
        # By default, start looking for new mappings from the beginning of the list, iterating downwards
        prevIndex = -1
        # Check if there is an existing mapping to start from
        if inputName in self.inputToBlimpMap:
            # Mapping exists
            blimpID = self.inputToBlimpMap[inputName]
            self.updateMapping(inputName, blimpID)
            prevIndex = blimpIDs.index(blimpID)
        validMappingFound = False
        # Iterate through blimps to find an unmapped blimp to map to
        while prevIndex < numBlimps-1:
            nextIndex = prevIndex + 1
            nextBlimpID = blimpIDs[nextIndex]
            if nextBlimpID in self.blimpToInputMap:
                # Mapping exists... move on
                prevIndex = nextIndex
            else:
                # Mapping doesn't exist... create it!
                self.updateMapping(inputName, nextBlimpID)
                validMappingFound = True
                break
        if not validMappingFound:
            # print("No valid blimps to map to.")
            pass

    def clearMappings(self):
        self.inputToBlimpMap.clear()
        self.blimpToInputMap.clear()
