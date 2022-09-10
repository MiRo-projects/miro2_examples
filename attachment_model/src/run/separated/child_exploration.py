#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../../')

# import subscription modules
from actions.explore import ChildExplore
from subscribers.action_sub import ChildAction

class ChildExploration:

    def __init__(self):
        rospy.init_node('child_exploration')
        self.explore = ChildExplore()
        self.action = ChildAction()

    def run(self):
        # to explore
        while not rospy.is_shutdown():
            if self.action.child == 0:
                self.explore.explore()

if __name__ == "__main__":
    exploration = ChildExploration()
    exploration.run()