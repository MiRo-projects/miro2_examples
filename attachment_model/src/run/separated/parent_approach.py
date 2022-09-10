#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../../')

# import subscription modules
from actions.approach import ParentApproach
from subscribers.action_sub import ParentAction   # subscribe from one controller only

class ParentExploration:

    def __init__(self):
        rospy.init_node('parent_approach')
        self.parent_approach = ParentApproach()
        self.action = ParentAction()

    def run(self):
        # to explore
        while not rospy.is_shutdown():
            if self.action.parent == 0:
                self.parent_approach.approach()

if __name__ == "__main__":
    approach = ParentApproach()
    approach.run()