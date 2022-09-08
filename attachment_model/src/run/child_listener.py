#!/usr/bin/env python3

# import core modules
import rospy

# set sys path for getting modules
import sys
sys.path.append('../')

# import module
from audio_detection.actions import ChildListening

class Listener():

    def __init__(self):
        rospy.init_node("child_listener")
        self.listener = ChildListening()

if __name__ == "__main__":
    listener = Listener()
    listener.listener.action()