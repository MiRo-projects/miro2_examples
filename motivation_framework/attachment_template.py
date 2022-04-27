import numpy as np
from enum import Enum

class Action(Enum):
    APPROACH = 1
    EXPLORE = 2

class ChildController:

    def __init__(self):
        self.currentAction = None
        self.t = 0.0
        self.dp = 0.0
        self.de = 0.0

    def evolveModel(self, emotional_distance, physical_distance):
        None

    def currentAction(self):
        None

    def updateEmotionalDistance( self, val ):
        None

    def updatePhysicalDistance( self, val ):
        None

class CarerController:

    def __init__(self):
        self.currentAction = None

    def evolveModel(self):
        None
    
    def currentAction(self):
        None

    def updateEmotionalDistance( self, val ):
        None

    def updatePhysicalDistance( self, val ):
        None




