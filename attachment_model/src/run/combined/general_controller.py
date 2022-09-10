#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting modules
import sys
sys.path.append('../../')

# import modules
from controller.robot_controller import BothController

class BothControl():
    
    def __init__(self):
        rospy.init_node("simultaneous_controller")
        self.controller = BothController()

    def run(self):
        self.controller.central_control()

if __name__ == "__main__":
    controller = BothControl()
    controller.run()