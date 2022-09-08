#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../')

# import subscription modules
from actions.explore import ParentExplore
from subscribers.action_sub import ParentAction

class ParentExploration:

    def __init__(self):
        rospy.init_node('child_exploration')
        self.parent_explore = ParentExplore()
        self.parent_action = ParentAction()

    def run(self):
        # to explore
        while not rospy.is_shutdown():
            if self.parent_action.parent == 1:
                self.parent_explore.explore()

if __name__ == "__main__":
    exploration = ParentExploration()
    exploration.run()