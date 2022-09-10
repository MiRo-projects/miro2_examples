#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting modules
import sys
sys.path.append('../../')

# import modules
from controller.robot_controller import ParentController

class ParentControl():
    
    def __init__(self):
        rospy.init_node("parent_controller")
        self.controller = ParentController(0, 0)

    def run(self):
        self.controller.central_control()

if __name__ == "__main__":
    controller = ParentControl()
    controller.run()