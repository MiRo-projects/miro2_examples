#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../../')

# import subscription modules
from actions.explore import ParentExplore
from subscribers.action_sub import ParentAction
from sign_stimuli.sign_stimuli import Client

class ParentExploration:

    def __init__(self):
        rospy.init_node('child_exploration')
        self.parent_explore = ParentExplore()
        self.action = ParentAction()
        self.still_explore = Client()

    def run(self):
        # to explore
        while not rospy.is_shutdown():
            if self.action.parent == 0:
                self.parent_explore.explore()
    
    def still_run(self):
        while not rospy.is_shutdown():
            if self.action.parent == 0:
                print("Looking for child")
                self.still_explore.loop()

if __name__ == "__main__":
    exploration = ParentExploration()
    exploration.still_run()