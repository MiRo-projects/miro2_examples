#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting subscriber modules
import sys
sys.path.append('../../')

# import subscription modules
from actions.approach import ChildApproach
from subscribers.action_sub import ChildAction  # subscribe from one controller only
from attachment_model.msg import Care           # bool for check which action to do

class ChildApproacher:

    def __init__(self):
        rospy.init_node('child_approach')
        self.approach = ChildApproach()
        self.action = ChildAction()
        # initialise publisher
        self.pub = rospy.Publisher(
            '/child/approacher_action', Care, queue_size= 0
        )
        self.sound = Care()

    def run(self):
        # to explore
        while not rospy.is_shutdown():
            if self.action.child == 1:
                if self.approach.condition_satisfied == True:
                    self.sound.initiating = True
                else:
                    self.approach.approach()
                    self.sound.initiating = False
            else:
                self.approach.condition_satisfied = False
                self.sound.initiating = False
            self.pub.publish(self.sound)

if __name__ == "__main__":
    approach = ChildApproacher()
    approach.run()