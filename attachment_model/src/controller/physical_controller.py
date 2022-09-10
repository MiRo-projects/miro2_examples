#!/usr/bin/env python3

# import core modules
from abc import abstractmethod
import rospy
import numpy as np

# import some other modules from within this package
from subscribers.child_pos_sub import ChildPosition
from subscribers.parent_pos_sub import ParentPosition

"""
    Parent class for calculating distance
"""
class PhysicalController(object):

    def __init__(self):
        self.dp = 0
        self.x_1 = 0
        self.y_1 = 0
        self.x_2 = 0
        self.y_2 = 0
    
    @abstractmethod
    def update_distance(self):
        pass

    """
        Reset all distances
    """
    def reset(self):
        self.x_1 = 0
        self.y_1 = 0
        self.x_2 = 0
        self.y_2 = 0

    """
        Calculate distance between two points
    """
    def calculate_physical_distance(self):
        # Use of pythagoras theorem
        x = self.x_1 - self.x_2
        y = self.y_1 - self.y_2
        x_2 = np.square(x)
        y_2 = np.square(y)
        p_d = np.sqrt(x_2 + y_2)
        return p_d

    """
        Calculate physical distance
    """
    def physical_distance(self):
        # Update distance
        self.update_distance()

        # Calculate physical distance
        self.dp = self.calculate_physical_distance()
        self.dp = self.dp/10
        rospy.loginfo("Updated physical distance: " + self.dp)
        if self.dp > 1:
            self.dp = 1

"""
    Sub-class to update necessary distance of both parent and child
"""
class BothPhysicalController(PhysicalController):

    def __init__(self):

        # Child position initialisation
        self.child_position = ChildPosition()
        self.x_1 = self.child_position.pos_x
        self.y_1 = self.child_position.pos_y

        # Parent position initialisation
        self.parent_position = ParentPosition()
        self.x_2 = self.parent_position.pos_x
        self.y_2 = self.parent_position.pos_y

        super().__init__()

    """
        Update both pairs of x and y with child and parent position
    """
    def update_distance(self):
        # Child position update
        self.x_1 = self.child_position.pos_x
        self.y_1 = self.child_position.pos_y

        # Parent position update
        self.x_2 = self.parent_position.pos_x
        self.y_2 = self.parent_position.pos_y

"""
    Sub-class to update necessary distance of parent
"""
class ParentPhysicalController(PhysicalController):

    def __init__(self):

        # Parent position initialisation
        self.parent_position = ParentPosition()
        self.x_2 = self.parent_position.pos_x
        self.y_2 = self.parent_position.pos_y

    """
        Update a pair of x and y with parent position
    """
    def update_distance(self):

        # Parent position update
        self.x_2 = self.parent_position.pos_x
        self.y_2 = self.parent_position.pos_y

"""
    Sub-class to update necessary distance of child
"""
class ChildPhysicalController(PhysicalController):

    def __init__(self):

        # Child position initialisation
        self.child_position = ChildPosition()
        self.x_1 = self.child_position.pos_x
        self.y_1 = self.child_position.pos_y

        super().__init__()

    """
        Update a pair of x and y with child position
    """
    def update_distance(self):

        # Child position update
        self.x_1 = self.child_position.pos_x
        self.y_1 = self.child_position.pos_y
    
