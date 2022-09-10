#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../../')

# import subscription modules
from actions.explore import ParentExplore
from subscribers.action_sub import BothAction   # subscribe from one controller only

class ParentExploration:

    def __init__(self):
        rospy.init_node('parent_exploration')
        self.parent_explore = ParentExplore()
        self.action = BothAction()

    def run(self):
        # to explore
        while not rospy.is_shutdown():
            if self.action.parent == 1:
                self.parent_explore.explore()

if __name__ == "__main__":
    exploration = ParentExploration()
    exploration.run()