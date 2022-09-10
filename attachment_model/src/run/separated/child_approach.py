#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../../')

# import subscription modules
from actions.approach import ChildApproach
from subscribers.action_sub import ChildAction   # subscribe from one controller only

class ChildApproacher:

    def __init__(self):
        rospy.init_node('child_approach')
        self.child_approach = ChildApproach()
        self.action = ChildAction()

    def run(self):
        # to explore
        while not rospy.is_shutdown():
            if self.action.child == 0:
                self.child_approach.approach()

if __name__ == "__main__":
    approach = ChildApproacher()
    approach.run()