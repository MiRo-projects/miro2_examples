# set to parent directory to import subscriber modules
#!/usr/bin/env python3

# import core python modules
from abc import abstractmethod
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../')

# import subscriber modules
from subscribers.child_listener_sub import ChildListener
from subscribers.parent_listener_sub import ParentListner

class EmotionController(object):

    def __init__(self):
        self.de = 0

    @abstractmethod
    def emotional_distance(self):
        pass

class BothEmotionController(EmotionController):

    def __init__(self):
        self.child_listener = ChildListener()
        self.parent_listener = ParentListner()
        super().__init__()

    def emotional_distance(self):
        child_care = self.child_listener.care
        parent_care = self.parent_listener.care
        if child_care and parent_care:
            rospy.loginfo('Both satisfied')
            self.de = 0
        else:
            self.de += (100-self.de)*0.005 
        self.de = self.de/100

class ParentEmotionController(EmotionController):

    def __init__(self):
        self.parent_listener = ParentListner()
        super().__init__()

    def emotional_distance(self):
        parent_care = self.parent_listener.care
        if parent_care:
            rospy.loginfo('Parent satisfied')
            self.de = 0
        else:
            self.de += (100-self.de)*0.005 
        self.de = self.de/100

class ChildEmotionController(EmotionController):

    def __init__(self):
        self.child_listener = ChildListener()
        super().__init__()

    def emotional_distance(self):
        child_care = self.child_listener.care
        if child_care:
            rospy.loginfo('Child satisfied')
            self.de = 0
        else:
            self.de += (100-self.de)*0.005 
        self.de = self.de/100