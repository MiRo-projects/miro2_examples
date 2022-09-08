#!/usr/bin/env python3
#
# The following class is supposed to decide on a list of listening actions of the MiRo
# depending on the state it is in after listening

import sys
import rospy
from abc import abstractmethod
from sound_recognition import RosCooDetection
from client_drive_voice import controller

class ListeningActions(object):
    
    """
        CONSTRUCTOR:
        threshold1 => first threshold to change action
        threshold2 => second threshold to change action
    """
    def __init__(self, starting_accumulation):
        rospy.init_node("Listener")
        self.threshold1 = 0.05
        self.threshold2 = 0.12
        self.timer = 2
        self.speaker = controller(sys.argv[1:])
        self.listener = RosCooDetection()
        self.current_accumulation = starting_accumulation
        self.listener.set_accumulation(starting_accumulation)

    """
        Initial action when it is below threshold 1
    """
    def action_init(self):
        self.current_accumulation = self.listener.listening(save=False)

    """
        Decide on the action after threshold one has been exceeded
    """
    def action_one(self):
        # make sound
        self.speaker.loop()
        # rest
        rospy.sleep(1)
        # start listening
        self.current_accumulation = self.listener.listening(save = False)
    
    """
        Decide on the action after threshold two has been exceeded
    """
    @abstractmethod
    def action_two(self):
        pass

    """
        Define the actions being done at each threshold
    """
    def action(self):
        while not rospy.is_shutdown():
            if self.current_accumulation <= self.threshold1:
                print("Stuck below threshold 0.05: " + str(self.current_accumulation))
                self.action_init()
            elif self.current_accumulation >= self.threshold2:
                self.action_two()
            else:
                print("Threshold 0.05 exceeded: " + str(self.current_accumulation))
                self.action_one()
    
class ChildListening(ListeningActions):

    def __init__(self):
        starting_accumulation = 0.06
        super().__init__(starting_accumulation)

    def action_two(self):
        print("Happy child: " + self.current_accumulation)
class ParentListening(ListeningActions):
    
    def __init__(self):
        starting_accumulation = 0.05
        super().__init__(starting_accumulation)

    def action_two(self):
        print("Happy parent")

if __name__ == "__main__":
    child = ChildListening()    
    child.action()

