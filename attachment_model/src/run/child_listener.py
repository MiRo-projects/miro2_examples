#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting modules
import sys
sys.path.append('../')

# import module
from audio_detection.actions import ChildListening
from subscribers.approacher_action_sub import ChildApproacherAction

class Listener():

    def __init__(self):
        rospy.init_node("child_listener")
        self.listener = ChildListening()
        # initialise publisher
        self.sub = ChildApproacherAction()
    
    def run(self):
        while not rospy.is_shutdown():
            print(self.sub.care)
            if self.sub.care is True:
                self.listener.action()
            else:
                self.listener.action_inactive()

if __name__ == "__main__":
    listener = Listener()
    listener.run()