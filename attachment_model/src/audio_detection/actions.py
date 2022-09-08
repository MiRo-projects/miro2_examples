#!/usr/bin/env python3
#
# The following class is supposed to decide on a list of listening actions of the MiRo
# depending on the state it is in after listening

import sys
import rospy
from abc import abstractmethod
from sound_recognition import RosCooDetection
from client_drive_voice import controller
from attachment_model.msg import Care

class ListeningActions(object):
    
    """
        CONSTRUCTOR:
        threshold1 => first threshold to change action
        threshold2 => second threshold to change action
    """
    def __init__(self, starting_accumulation):
        self.threshold1 = 0.05
        self.threshold2 = 0.12
        self.timer = 2
        self.speaker = controller(sys.argv[1:])
        self.listener = RosCooDetection()
        self.current_accumulation = starting_accumulation
        self.listener.set_accumulation(starting_accumulation)

    """
        Initial action when it is below threshold one
    """
    def action_init(self):
        self.current_accumulation = self.listener.listening(save=False)
        self.care.initiating = False

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
        self.care.initiating = False
    
    """
        Publush care be initialised to be true when threshold two has been exceeded
    """
    def action_two(self):
        self.care.initiating = True

    """
        Define the actions being done at each threshold
    """
    def action(self):
        while not rospy.is_shutdown():
            # see if care needs to be given
            if self.current_accumulation <= self.threshold1:
                print("Stuck below threshold 0.05: " + str(self.current_accumulation))
                self.action_init()
            elif self.current_accumulation >= self.threshold2:
                self.action_two()
            else:
                print("Threshold 0.05 exceeded: " + str(self.current_accumulation))
                self.action_one()
            # publish care after each iteration
            self.care_detection.publish(self.care)

# Classes are inherited to set some default values for the constructor    
class ChildListening(ListeningActions):

    def __init__(self):
        rospy.init_node("child_listener")
        starting_accumulation = 0.00
        self.care_detection = rospy.Publisher('/child/care_detection', Care, queue_size=0)
        self.care = Care()
        super().__init__(starting_accumulation)

class ParentListening(ListeningActions):
    
    def __init__(self):
        rospy.init_node("parent_listener")
        starting_accumulation = 0.06
        self.care_detection = rospy.Publisher('/parent/care_detection', Care, queue_size=0)
        self.care = Care()
        super().__init__(starting_accumulation)
