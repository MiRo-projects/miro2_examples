#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting modules
import sys
sys.path.append('../../')

# import modules
from controller.robot_controller import ChildController

class ChildControl():
    
    def __init__(self):
        rospy.init_node("child_controller")
        self.controller = ChildController(0,0)

    def run(self):
        self.controller.central_control()

if __name__ == "__main__":
    controller = ChildControl()
    controller.run()